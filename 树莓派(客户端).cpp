#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

//小车控制函数
void turnRight() //右转
{
}
void turnLeft() //左转
{
}
void run() //直行
{
}
void stop() //停止
{
}
void controlcar(int stopflag, int controlflag)
{
    if (stopflag == 0)
        stop();
    else
    {
        if (controlflag == 0)
            turnLeft();
        if (controlflag == 1)
            run();
        if (controlflag == 2)
            turnRight();
        else
            stop();
    }
}

//创造客户端socket，输入IP地址和端口号
int sockfd;
struct sockaddr_in serv_addr;
void creatsocket(char *ipAdrr, char *port)
{
    int portno;

    struct hostent *server;

    portno = atoi(port);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(ipAdrr); //这里填IP地址
    if (server == NULL)
    {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR connecting");
}

int main()
{
    creatsocket("127.0.0.1", "11332");
    VideoCapture cap(0);
    Mat frame;
    char controlFlag[1024];
    while (true)
    {
        bzero(controlFlag, 1024);
        cap >> frame;
        if (frame.empty())
        {
            cout << "can't give video" << endl;
            break;
        }
        pyrDown(frame, frame, Size(frame.cols / 2, frame.rows / 2));

        // Send data here
        for (int i = 0; i < 5; i++)
            int bytes = send(sockfd, frame.data, frame.total() * frame.elemSize(), 0); //

        int bytes = send(sockfd, frame.data, frame.total() * frame.elemSize(), 0);

        //recive data here
        int rec_num = recv(sockfd, controlFlag, 1024, 0);
        controlFlag[rec_num] = '\0';

        int stopflag = (int)(controlFlag[0] - '0');
        int controlflag = (int)(controlFlag[1] - '0');
        printf("stopflag:%d  controlflag:%d\n", stopflag, controlflag);
        controlcar(stopflag, controlflag);

        if (waitKey(30) >= 0)
            break;
    }
    cap.release();
    shutdown(sockfd, SHUT_RDWR);
    return 0;
}