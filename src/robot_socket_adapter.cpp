#include "robot_socket_adapter.h"

// =============================================================================
// STRING MANIPULATIONS
// =============================================================================

std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r_ ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

// =============================================================================
// FUNCTIONS
// =============================================================================

RobotSocketAdapter::RobotSocketAdapter()
    : connected(false)
{

}

RobotSocketAdapter::~RobotSocketAdapter()
{
    this->disconnect();
}


bool RobotSocketAdapter::connect(const char* host, const unsigned int port)
{
    #ifdef _WIN32
        WSAStartup(MAKEWORD(2,0), &WSAData);
    #endif

    server_socket.sin_addr.s_addr = inet_addr(host);
    server_socket.sin_family	  = AF_INET;
    server_socket.sin_port		  = htons(port);

    //sock = socket(AF_INET, SOCK_DGRAM , IPPROTO_UDP);
	sock = socket(AF_INET, SOCK_STREAM, 0);

    #ifdef _WIN32
        connected = connected = (::connect(sock, (SOCKADDR*)&sin, sizeof(sin)) != SOCKET_ERROR);
    #elif __linux__
        auto res = (::connect(sock, (struct sockaddr*)&server_socket, sizeof(server_socket))<0);
        connected = (res == 0);
        usleep(50*1000);
    #endif


    return connected;
}

void RobotSocketAdapter::disconnect()
{
    if (this->connected){
        //dtor
        #ifdef _WIN32
            closesocket(sock); // Fermeture du socket
            WSACleanup();
        #elif __linux__
            close(sock); // Fermeture du socket
        #endif

        connected = false;
    }
}

const bool RobotSocketAdapter::sendCmd(std::string cmd, std::vector<double> args)
{
    std::string msg = cmd;
    for (auto i = 0; i < args.size(); i++){
        msg.append(",");
        msg.append( std::to_string(args[i]) );
    }
    std::cout << msg << std::endl;

    char buffer[1024];
    strncpy(buffer, msg.c_str(), msg.size());
    ::send(sock,buffer,msg.size(),0);

    char bufferResponse[500];
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    //std::cout << "response from visa" << str << std::endl;
    rtrim(str);
    if (str.compare(0,2,"OK") == 0){
        return true;
    }
    else{
        std::cerr << "ERROR: " << bufferResponse << std::endl;
        return false;
    }
}

const bool RobotSocketAdapter::setJointPosAbs(std::vector<double> joints)
{
    return sendCmd("SETJOINTPOSABS",joints);
}

const bool RobotSocketAdapter::setJointPosRel(std::vector<double> joints)
{
    return sendCmd("SETJOINTPOSREL",joints);
}

const bool RobotSocketAdapter::setJointVel(std::vector<double> velocities)
{
    return sendCmd("SETJOINTVEL",velocities);
}

const bool RobotSocketAdapter::homing()
{
    return sendCmd("HOMING",{});
}

void RobotSocketAdapter::getJointPos(std::vector<double> & values)
{
    char buffer[12] = "GETJOINTPOS";
    char bufferResponse[500]; //too large but sure to fit
    ::send(sock, buffer,sizeof(buffer)-1,0);
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    rtrim(str);
    std::vector<std::string> valuesStr = split(str, ',');

    values.clear();
    values.resize(valuesStr.size());
    for (auto i = 0; i < values.size(); i++){
        values[i] = std::stof(valuesStr[i]);
    }
}

void RobotSocketAdapter::getToolTransform(std::vector<double> & matrix)
{
    char buffer[11] = "GETTOOLPOS";
    char bufferResponse[500]; //too large but sure to fit
    ::send(sock, buffer,sizeof(buffer)-1,0);
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    rtrim(str);
    std::vector<std::string> valuesStr = split(str, ',');

    matrix.clear();
    matrix.resize(valuesStr.size());
    for (auto i = 0; i < matrix.size(); i++){
        matrix[i] = std::stof(valuesStr[i]);
    }
}

std::string RobotSocketAdapter::getImage()
{
    char buffer[9] = "GETIMAGE";
    char bufferResponse[500]; //UDP max package size
    char bufferImage[1000000]; //UDP max package size
    std::string msgPrefix = "PACKAGE_LENGTH:";

    ::send(sock, buffer, 8, 0);
    ::recv(sock, bufferResponse, 500, 0);

	std::string message(bufferResponse);
	message = message.substr(msgPrefix.size(),10);
	message.erase(std::remove_if(message.begin(), message.end(),
                        [](char c) { return !std::isdigit(c); }),
         message.end());
	int imageSize = std::stoi(message); //parse int
	//std::cout << "imageSize: " << imageSize << std::endl;

	//acquire the image
	::recv(sock, bufferImage, imageSize+1, MSG_WAITALL);

	//delete prefix and decode
	std::string encodedImage(bufferImage);
	encodedImage = encodedImage.substr(0,imageSize);

    encodedImage.erase(0,22);
    return base64_decode(encodedImage);
}

#ifdef WITH_OPENCV
cv::Mat RobotSocketAdapter::getImageOpenCV()
{
    std::string decodedImage = this->getImage();

    std::vector<uchar> vectordata(decodedImage.begin(),decodedImage.end());
    cv::Mat data_mat(vectordata,true);

    cv::Mat image(cv::imdecode(data_mat,1)); //put 0 if you want greyscale
    return image;
}
#endif

#if defined(WITH_OPENCV) && defined(WITH_VISP)
vpImage<unsigned char> RobotSocketAdapter::getImageViSP()
{
    vpImage<unsigned char> I;
    vpImageConvert::convert(this->getImageOpenCV(), I);
    return I;
}
#endif
