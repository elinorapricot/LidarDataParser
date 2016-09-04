#include "lidar.h"
#include <cassert>
#include <iostream>

using namespace std;

lidar::lidar(int port, int timeout)
{
    this->timeout = timeout;
    addrlen = sizeof(remaddr);
    this->port = port;
    int erno;

    memset(buf, 0, BUFSIZE);

    if (erno = init()) std::cout << "Error " << erno << " at UDP init" << std::endl;
    assert(!erno);
}

lidar::~lidar()
{
}

int lidar::init()
{
    /* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) {
        /* Tell the user that we could not find a usable */
        /* Winsock DLL.                                  */
        printf("WSAStartup failed with error: %d\n", err);
        return 1;
    }

    /* create a UDP socket */
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cout << WSAGetLastError() << std::endl;
        perror("cannot create socket\n");
        return 2;
    }

    /* bind the socket to any valid IP address and a specific port */
    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
        perror("bind failed"); return 3;
    }

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int));

    return 0;
}

bool lidar::isBigEndian(){
    unsigned short a = 0x1234;
    if (*((unsigned char *)&a) == 0x12)
        return true;
    else
        return false;
}

int lidar::CanGetRaw()
{
    char buffer[BUFSIZE] = { 0 };
    /*recieve data from UDP*/
    recvlen = recvfrom(fd, buffer, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);

    for (int i = 0; i < BUFSIZE; i++)
    {
        if (buffer[i] == 0) break;
        memcpy(buf, buffer + i, BUFSIZE - i);
        return 0;
    }
    return 1;
}

void lidar::process()
{
    //TODO timestamp, clear datatable when needed, find out when? :D
    int i = 0;
    int j = 0;
    int k = 0;
    firingBlock currentBlock;
    //check the 1206 byte of the current packet
    while(i < BUFSIZE){
        //first check only the 12 x 100 byte data (12 blocks firing data)
        while (i<1200){
            //check each block of firing data (one block contains one firing report of 32 lasers)
            while (j < 100){

                switch (j)
                {
                case 0:
                    currentBlock.blockID = buf[i];
                    break;
                case 1:
                    currentBlock.blockID += buf[i] << 8;
                    break;
                case 2:
                    currentBlock.degree = buf[i];
                    break;
                case 3:
                    currentBlock.degree += buf[i] << 8;
                    break;
                default:
                    if (j % 3 == 1){ 
                        firingData.distance = buf[i];
                    }
                    else if (j % 3 == 2){
                        firingData.distance += buf[i]<<8;
                    }
                    else {
                        firingData.intensity = buf[i];
                    }
                    break;
                }
                if (j % 3 == 0 && j>3){
                    currentBlock.data[k++] = firingData;
                    firingData.distance = 0;
                    firingData.intensity = 0;
                }
                i++;
                j++;
            }
            dataTable.push_back(currentBlock);
            currentBlock.blockID = 0;
            currentBlock.degree = 0;
            memset(currentBlock.data, 0, k);
            j = 0;
            k = 0;
            
        }
        //handle 6 information bytes in the end of the packet 
        //(4 bytes for GPS timestamp, 1 byte for status type 1 byte for status value)

        //convert and save 4 bytes of GPS timestamp to one variable
        if (isBigEndian()){
            memcpy(&timestamp, &(buf[1203]), 4);
        }
        //in case of little endian, we have to swap the 4 bytes
        else{
            for (int k = 0; k < 4; k++){
                timestamp += buf[1203 - k] << (8 * k);
            }
        }
        for (int l = dataTable.size() - 12; l < dataTable.size(); l++){
            dataTable.at(l).timestamp = timestamp;
        }
        //create an associative array (map) from the status type and status information
        statusData[buf[1204]] = buf[1205];
        i++;
    }
    memset(buf, 0, BUFSIZE);
}


