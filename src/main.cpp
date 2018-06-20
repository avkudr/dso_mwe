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
#include "distances.h"
#include "concentric_tube_robot.h"
#include "path_planner.hpp"

// I have no idea why it is needed, but in order to have a real displacement of
// the camera, I had to devide camera translation by it. ...May be smth is wrong
// with calibration matrix.
#define WEIRD_SCALE_FACTOR 27.758813860811210

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

using Vector3DPoints = std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>;
using Vector2DPoints = std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>>;

double sigmoid(double lambda, double F){
	return (2-2./(1+exp(-lambda*fabs(F))));
}

Eigen::Vector3d getRotationTaskVels(double l1, double l2, double l3, double k2,double F){
	Eigen::Matrix3d J;
	J(0,0) = -cos(k2*l2)/sin(k2*l2);
	J(0,1) = 1;
	J(0,2) = (k2*(k2*l3*cos(k2*l2)*cos(k2*l2) + k2*l3*sin(k2*l2)*sin(k2*l2)))/sin(k2*l2);

	J(1,0) = -cos(k2*l2)/sin(k2*l2);
	J(1,1) = 1;
	J(1,2) = k2 + (k2*(k2*l3*cos(k2*l2)*cos(k2*l2) + k2*l3*sin(k2*l2)*sin(k2*l2)))/sin(k2*l2);

	J(2,0) = 1/sin(k2*l2) - cos(k2*l2)/sin(k2*l2);
	J(2,1) = 1;
	J(2,2) = k2 + (k2*(k2*l3*cos(k2*l2)*cos(k2*l2) + k2*l3*sin(k2*l2)*sin(k2*l2)))/sin(k2*l2) - (k2*(sin(k2*l2) + k2*l3*cos(k2*l2)))/sin(k2*l2);

	Eigen::Vector3d v;
	v << 0,0,F;

	return J*v;
}

int main( int argc, char** argv )
{

	/*************************** INIT *********************************/

	RobotSocketAdapter * adapter = new RobotSocketAdapter();
	adapter->connect();

	calib ="/home/akudryavtsev/Projects/dso_mwe/camera_visa2.txt";

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

		std::vector<double> joints;
		adapter->getJointPos(joints);
		ConcentricTubeRobot * virtualRobot = new ConcentricTubeRobot();
		virtualRobot->setJointPos(joints);

		PathPlanner * pathPlanner = new PathPlanner();
		pathPlanner->setCircleRadius(0.1);

		cv::Mat plot2d = Mat::zeros( 640, 480, CV_8UC3 );

		bool isMotionPlanningActive = false;
		bool isOperating;
		try{
			int ii = 0;
			isOperating = true;

			//getting initial robot transform
			std::vector<double> matrix;
			adapter->getToolTransform(matrix);
			Eigen::Matrix4d toolTransform(matrix.data());
			//std::cout << "Initial tool pose:\n " << toolTransform << std::endl;

			//move robot forward to initialize SLAM
			//adapter->setJointVel({0,0,0,0,0,0.0005});
			bool isSLAMInitDone = false;

			Eigen::Matrix4d initialToolTransform(toolTransform);
			Eigen::Vector3d offset3D;
			offset3D << toolTransform(0,3),toolTransform(1,3),toolTransform(2,3);
			Eigen::Vector2d offset;
			offset << toolTransform(0,3),toolTransform(2,3); //only x and z
			Eigen::Vector2d zero;
			zero << 0,0;

			Vector2DPoints toolPath;
			toolPath.clear();
			toolPath.push_back(-offset + offset);
			toolPath.push_back(   zero + offset);

			//mat.conservativeResize(mat.rows(), mat.cols()+1);
			//mat.col(mat.cols()-1) = vec;

			while( isOperating ) {

				//printf("\n-- START OF FRAME %d \n", ii);

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

				printf("\n**************************************");
				printf("**************************************\n");

				const auto & pcl = ow->pointCloud;
				std::cout << "Number of 3D points: "
						  << pcl.size() << std::endl;

				// get camera pose from slam
				Eigen::MatrixXd camPoseMat = fullSystem->camToWorld.matrix();

				// get points that are close to the plane parallel to the image frame
				// but located at a distance d=0.5 from camera. Point is considered as close if
				// the distance between it and a plane is lower than 0.2

				pathPlanner->init(pcl,camPoseMat);
				pathPlanner->projectClosePointsOnPlane3D();
				pathPlanner->project3DPointsto2D();

				double lambda = 0.0005;
				double F = lambda * pathPlanner->getRepulsiveForceZ();

				adapter->getJointPos(joints);
				virtualRobot->setJointPos(joints);


				double beta = sigmoid(10,2*F);
				std::cout << "beta: " << beta << std::endl;
				Eigen::Vector3d v1, v2;
				v1 << 0.0004,0.0004,0.0005;

				double l1 = virtualRobot->getSegmentLength(0);
				double l2 = virtualRobot->getSegmentLength(1);
				double l3 = virtualRobot->getSegmentLength(2);
				double k2 = virtualRobot->getSegmentCurvature(1);
				v2 = getRotationTaskVels(l1,l2,l3,k2,F);

				if (beta > 0.7) beta = 1;
				if (beta < 0.3) beta = 0.3;
				auto v = beta*v1 + 0.000008 * (1-beta)*v2;

				adapter->setJointVel({0,0,0,v(0),v(1),v(2)});

				printf("**************************************");
				printf("**************************************\n\n");

				//--------------- Additional plots -----------------------------

				const auto & closePointsProjections = pathPlanner->getClosePointsProjectedOnPlane2D();
				const auto & endEffectorPoint       = pathPlanner->getDesiredPoint2D();

				std::cout << "Number of points close to plane : " << closePointsProjections.cols() << std::endl;
				std::cout << "End-effector des pos: (" << endEffectorPoint.x() << ";" << endEffectorPoint.y() << ")"<< std::endl;
				plot2d = 100 * Mat::ones( 480, 640, CV_8UC3 );
				int scale = 200;
				for (auto i = 0; i < closePointsProjections.cols(); i++){
					double x = scale*(closePointsProjections(0,i)-endEffectorPoint.x()) + 640/2;
					double y = scale*(closePointsProjections(1,i)-endEffectorPoint.y()) + 480/2;
					cv::circle(plot2d, Point(x,y),1, Scalar(255,0,255),CV_FILLED,2,0);
				}
				cv::circle(plot2d, Point(640/2,480/2),1, Scalar(255,255,0),CV_FILLED,2,0);
				cv::circle(plot2d, Point(640/2,480/2),
					scale*pathPlanner->getCircleRadius(), Scalar(255,255,0),0,2,0);
				cv::line(plot2d, Point(640/2,480/2), Point(640/2,480/2+scale*F), Scalar(255,0,0));
				cv::imshow("Image",plot2d);
				cv::waitKey(1);

				//--------------------------------------------------------------

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
