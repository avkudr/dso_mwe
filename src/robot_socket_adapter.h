#ifndef RobotUDPAdapter_H
#define RobotUDPAdapter_H

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <vector>
#include <string>
#include <string.h>

#include <cpp-base64/base64.h>

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#endif

#ifdef WITH_VISP
#include <visp3/io/vpImageIo.h>
#endif

#ifdef _WIN32

#include <string>
#include <winsock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")

#elif __linux__

#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#endif

class RobotSocketAdapter
{
    public:

        RobotSocketAdapter();
        ~RobotSocketAdapter();
        bool connect(const char* = "127.0.0.1", const unsigned int = 1234);
        void disconnect();
        const bool isConnected(){ return connected; }

        const bool setJointPosAbs(std::vector<double>);
        const bool setJointPosRel(std::vector<double>);
        const bool setJointVel(std::vector<double>);
        const bool homing();

        void getJointPos(std::vector<double> & );

        std::string getImage();

        #ifdef WITH_OPENCV
            cv::Mat getImageOpenCV();
        #endif

        #if defined(WITH_OPENCV) && defined(WITH_VISP)
            vpImage<unsigned char> getImageViSP();
        #endif

    private:
        #ifdef _WIN32
            WSADATA WSAData; // configuration socket
            SOCKET sock;
            SOCKADDR_IN sin;
        #elif __linux__ || __APPLE__
            int sock;
            struct sockaddr_in server_socket;
        #endif

        const bool sendCmd(std::string, std::vector<double>);

        bool connected;
        bool isVelCtrlActive; // not used yet
};
#endif // RobotUDPAdapter_H
