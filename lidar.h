#pragma once


#define WIN32_LEAN_AND_MEAN
#define _CRT_SECURE_NO_WARNINGS

#ifdef _WIN32
#pragma comment(lib, "Ws2_32.lib")
#include <io.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <WinSock2.h>
#include <Windef.h>
#else //TODO: find out what other headers needed on linux
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#endif

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <array>
#include <vector>


#define BUFSIZE 1206
struct firingInformation {
    unsigned short distance;
    char intensity;
};

struct firingBlock{
    int blockID;
    unsigned int timestamp;
    unsigned short degree;
    firingInformation data[32];
};

class lidar
{

public:
    int port;
    std::vector<firingBlock> dataTable;
    lidar(int, int);
    ~lidar();
    void process();
    int CanGetRaw();

private:
    int err;
    int timeout;                                           //milliseconds
    int recvlen;                                           //bytes received 
    int fd;                                                //our socket 
    unsigned char buf[BUFSIZE];                            //receive buffer 
    struct sockaddr_in myaddr;                             //our address 
    struct sockaddr_in remaddr;                            //remote address 
    socklen_t addrlen;                                     //length of addresses 
    WORD wVersionRequested;
    WSADATA wsaData;
    unsigned int timestamp;
    unsigned short blockID;
    unsigned short degree;
    struct firingInformation firingData;
    std::map<unsigned char, unsigned char> statusData;

    int init();
    bool isBigEndian();
};

