/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"


#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include "robot_socket_adapter.h"

std::string vignette = "";
std::string gammaCalib = "";
std::string source = "";
std::string calib = "";
double rescale = 1;
bool reverse = false;
bool disableROS = false;
int start=0;
int end=100000;
bool prefetch = false;
float playbackSpeed=0;	// 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload=false;
bool useSampleOutput=true;

int mode=0;

bool firstRosSpin=false;

using namespace dso;
using namespace cv;

Mat camframe, gray;

void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	firstRosSpin=true;
	while(true) pause();
}

int main( int argc, char** argv )
{

	/*************************** INIT *********************************/

	RobotSocketAdapter * adapter = new RobotSocketAdapter();
	adapter->connect();

	calib ="/home/akudryavtsev/Projects/dso_mwe/camera_hercules.txt";

	setlocale(LC_ALL, "");

	mode=1;
	printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
	setting_photometricCalibration = 0;
	setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
	setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).

	// hook crtl+C.
	boost::thread exThread = boost::thread(exitThread);

	ImageFolderReader* reader = new ImageFolderReader(source,calib, gammaCalib, vignette);
	reader->setGlobalCalibration();

	if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
	{
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}

	int lstart = start;
	int lend = end;
	int linc = 1;
	if(reverse)
	{
		printf("REVERSE!!!!");
		lstart=end-1;
		if(lstart >= reader->getNumImages())
		lstart = reader->getNumImages()-1;
		lend = start;
		linc = -1;
	}

	FullSystem* fullSystem = new FullSystem();
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed==0);

	IOWrap::PangolinDSOViewer* viewer = 0;
	if(!disableAllDisplay)
	{
		viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], false);
		fullSystem->outputWrapper.push_back(viewer);
	}

	auto ow = new IOWrap::SampleOutputWrapper();
	fullSystem->outputWrapper.push_back(ow);



	/*
	* All the magin happens here.
	*
	*/
	std::thread runthread([&]() {

		bool isOperating;
		try{
			int ii = 0;
			isOperating = true;

			//move robot forward to initialize SLAM
			adapter->setJointVel({0,0,0,0,0,0.001});
			bool isSLAMInitDone = false;

			while( isOperating ) {

				printf("\n-- START OF FRAME %d \n", ii);

				if(!fullSystem->initialized){	// if not initialized: reset start time.
					std::cout << "SLAM: init phase..." << std::endl;
				}else{
					if (!isSLAMInitDone){
						std::cout << "SLAM: init phase... Done" << std::endl;
						isSLAMInitDone = true;
						adapter->setJointVel({0,0,0,0,0,0});
					}
				}

				int i = ii;

				//Acquire image
				ImageAndExposure* img;
				img = new ImageAndExposure(640,480,ii);

				camframe = adapter->getImageOpenCV();
				cvtColor(camframe, gray, COLOR_BGR2GRAY);

				for(int k=0; k<gray.rows; k++){
					for(int j=0; j<gray.cols; j++){
						img->image[(k*640)+j] = gray.at<uchar>(k,j);
					}
				}

				//SLAM
				fullSystem->addActiveFrame(img, i);
				delete img;

				if(fullSystem->initFailed || setting_fullResetRequested)
				{
					std::ofstream myfile;
					myfile.open ("pointCould.m");
					myfile << "pcl = [ ";
					const auto & pcl = ow->pointCloud;
					for (auto i = 0; i < pcl.size(); i++){
						myfile << pcl[i] << ";" << std::endl;
					}
					myfile << "];" << std::endl;
					myfile.close();

					if(ii < 1000 || setting_fullResetRequested)
					{
						printf("RESETTING!\n");

						std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
						delete fullSystem;

						for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

						fullSystem = new FullSystem();
						fullSystem->setGammaFunction(reader->getPhotometricGamma());
						fullSystem->linearizeOperation = (playbackSpeed==0);
						fullSystem->outputWrapper = wraps;

						setting_fullResetRequested = false;
					}
				}

				if(fullSystem->isLost)
				{
					printf("LOST!!\n");
					break;
				}

				printf("\n**************************************\n");
				std::cout << "     Number of 3D points: "
						  << ow->pointCloud.size() << std::endl;
				printf("**************************************\n\n");

				const auto & cameraPose = fullSystem->camToWorld.matrix3x4();
				Eigen::Matrix<double,3,4> camPoseMat = fullSystem->camToWorld.matrix3x4();
				std::cout << "Camera Pose: " << std::endl
						  << camPoseMat << std::endl;
				Eigen::Vector4d zAxis;

				printf("**************************************\n\n");

				ii++;
			}

			fullSystem->blockUntilMappingIsFinished();


		}catch(std::exception const& e){
			//adapter->setJointVel({0,0,0,0,0,0});
			//std::cerr << e.what() << std::endl;
			//isOperating = false;
		}
	});


	if(viewer != 0)
	viewer->run();

	runthread.join();

	for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
	{
		ow->join();
		delete ow;
	}

	printf("DELETE FULLSYSTEM!\n");
	delete fullSystem;

	printf("DELETE READER!\n");
	delete reader;

	printf("EXIT NOW!\n");
	return 0;
}
