#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>

#include <fstream>
#include <sstream>
#include <iostream>
//#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace dnn;
using namespace std;

// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;        // Width of network's input image
int inpHeight = 416;       // Height of network's input image
int stopDistance = 20;     //在标志前20厘米停止
int stopflag = 1;          //小车停止标志 0停，1行进
int controlflag = 1;       //小车转弯标志 0左转，1前进，2右转
int imgSize = 691200 * 3;  //imgSize=frame.total()*frame.elemSize();
vector<string> classes;

//创建socket(服务端)
int client_socket;
int creatSocket(char *ipAdrr, int port)
{
    struct sockaddr_in server_addr;
    server_addr.sin_len = sizeof(struct sockaddr_in);
    server_addr.sin_family = AF_INET; //Address families AF_INET互联网地址簇
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ipAdrr);
    bzero(&(server_addr.sin_zero), 8);

    //创建socket
    int server_socket = socket(AF_INET, SOCK_STREAM, 0); //SOCK_STREAM 有连接
    if (server_socket == -1)
    {
        perror("socket error");
        return 1;
    }

    //绑定socket：将创建的socket绑定到本地的IP地址和端口，此socket是半相关的，只是负责侦听客户端的连接请求，并不能用于和客户端通信

    int bind_result = Bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (bind_result == -1)
    {
        perror("bind error");
        return 1;
    }

    //listen侦听 第一个参数是套接字，第二个参数为等待接受的连接的队列的大小，在connect请求过来的时候,完成三次握手后先将连接放到这个队列中，直到被accept处理。如果这个队列满了，且有新的连接的时候，对方可能会收到出错信息。
    if (listen(server_socket, 5) == -1)
    {
        perror("listen error");
        return 1;
    }

    struct sockaddr_in client_address;
    socklen_t address_len;
    client_socket = accept(server_socket, (struct sockaddr *)&client_address, &address_len);
    //返回的client_socket为一个全相关的socket，其中包含client的地址和端口信息，通过client_socket可以和客户端进行通信。
    if (client_socket == -1)
    {
        perror("accept error");
        return -1;
    }
}

//单目测距 函数模型详见 https://blog.csdn.net/sillykog/article/details/71214107
float knownDistance = 10;  //将A4纸放到距离摄像头10厘米的地方
float knownWidth = 8.27;   //A4纸的宽度
float knownHeight = 11.69; //A4纸的长度
float f = 0.3;             //摄像头焦距为3mm
float ypix = 0.1;          //像素长为1mm
float o3m = 5;             //图像坐标中心在y轴上的距离为5cm
float H = 12.0;            //摄像头高度
float h = 8.0;             //指示牌高度
float a = H / o3m;         //tanα
float calDistance(float imgcenter_y, float signcenter_y)
{
    float b = fabs(signcenter_y - imgcenter_y) * ypix / f; //tanγ
    float c = fabs(a - b) / (1 + a * b);                   //tanβ
    float d = (H / c) * (1 - h / H);                       //获得距离

    return d;
}

//道路检测函数
int directionControl(Mat img)
{
    //截取图像的下半部分，便于处理
    Rect rect(0, img.rows / 3, img.cols, 2 * img.rows / 3);
    Mat croppedImg = img(rect);

    //提取图像的H通道，屏蔽饱和度和明度的影响
    Mat hsv;
    cvtColor(croppedImg, hsv, COLOR_BGR2HSV);
    imshow("hsv", hsv);
    Mat hImg;
    hImg.create(hsv.size(), hsv.depth());
    int channels[] = {0, 0};
    mixChannels(&hsv, 1, &hImg, 1, channels, 1);
    imshow("himage", hImg);

    //二值化
    Mat binaryImg;
    threshold(hImg, binaryImg, 100, 255, THRESH_BINARY);
    imshow("binaryImg", binaryImg);

    ////腐蚀，去除噪点
    //Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    //erode(binaryImg, binaryImg, kernel);

    ////膨胀使白线更宽
    //Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    //for(int i=0;i<3;i++)
    //dilate(binaryImg, binaryImg, kernel);

    //提取两白线的中心点，进而算出路的中心点以控制小车方向
    Mat labels, stats, centriods;
    connectedComponentsWithStats(binaryImg, labels, stats, centriods);
    Point roadCenter = (stats.at<int>(1, CC_STAT_LEFT) < stats.at<int>(2, CC_STAT_LEFT)) ? Point(stats.at<int>(2, CC_STAT_LEFT) / 2 + stats.at<int>(1, CC_STAT_LEFT) / 2 + stats.at<int>(1, CC_STAT_WIDTH) / 2, 0) : Point(stats.at<int>(2, CC_STAT_LEFT) / 2 + stats.at<int>(1, CC_STAT_LEFT) / 2 + stats.at<int>(2, CC_STAT_WIDTH) / 2, 0);

    //输出图像
    cvtColor(binaryImg, binaryImg, COLOR_GRAY2BGR);
    circle(binaryImg, roadCenter, 2, Scalar(0, 0, 255), 10); //用红点表示路的中心
    Point imgCenter(binaryImg.cols / 2, 0);
    circle(binaryImg, imgCenter, 2, Scalar(0, 255, 255), 10); //用黄点表示图像的中心
    imshow("direction control", binaryImg);

    //判断输出返回值，0左转，1直行， 2右转。
    if (roadCenter.x < binaryImg.cols / 2)
        return 0;
    else if (roadCenter.x > binaryImg.cols / 2)
        return 2;
    else
        return 1;
}

// 图像后处理（画框和单目测距）;返回停止标志。
int drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame)
{
    int output = 1;
    float imgcenter_y = frame.rows / 2;               //图像的中心坐标
    float signHeight = bottom - ((bottom - top) / 2); //检测目标在图像中的Y坐标

    circle(frame, Point(frame.cols / 2, imgcenter_y), 2, Scalar(255, 0, 0), 2);
    circle(frame, Point(frame.cols / 2, signHeight), 2, Scalar(0, 255, 255), 2);
    float signDistance = calDistance(imgcenter_y, signHeight); //获取到目标的距离
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 255, 255), 1);

    //Get the label for the class name and its confidence
    string conf_label = format("%.2f", conf);
    string distance = format("%.2f", signDistance);
    string label = "";
    if (!classes.empty())
    {
        label = classes[classId] + ":" + conf_label + " " + distance + "cm";
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - labelSize.height), Point(left + labelSize.width, top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, LINE_AA);

    if (!classes[classId].compare("stop sign"))
    { //如果检测到stop sign标志
        if (signDistance < stopDistance)
            output = 0;
        else
            output = 1;
    }
    if (!classes[classId].compare("traffic light"))
    { //如果检测到traffic light标志
        if (signDistance < stopDistance)
            output = 0;
        else
            output = 1;
    }

    return output;
}

// 图像处理
void postprocess(Mat &frame, const vector<Mat> &outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float *data = (float *)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold) //获得置信度大于阈值的框
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        stopflag = drawPred(classIds[idx], confidences[idx], box.x, box.y,
                            box.x + box.width, box.y + box.height, frame);
    }
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net &net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        {
            names[i] = layersNames[outLayers[i] - 1];
        }
    }
    return names;
}
const char *keys = {
    "{heilp h usage?| |prin this message}"
    "{@video |  | Video file}"
    "{@classfile |/Users/luohonglin/Desktop/当前/opencv4_vscode/myData/yolo/coco.names.txt| 获取yolo目标检测的对象名文件} "
    "{@modelConfiguration | /Users/luohonglin/Desktop/当前/opencv4_vscode/myData/yolo/yolov2.cfg| yolov3配置文件}"
    "{@modelWeights |/Users/luohonglin/Desktop/当前/opencv4_vscode/myData/yolo/yolov2.weights| yolov3对象检测权重}"

};

int main(int argc, char **argv)
{

    creatSocket("127.0.0.1", 11332); //创建socket
    //获取输入信息
    CommandLineParser parser(argc, argv, keys); //获取main函数的输入和先前定义的keys常量

    string vedioFile = parser.get<string>(0);          //获取视频文件
    string classesFile = parser.get<string>(1);        //获取目标检测对象名
    String modelConfiguration = parser.get<string>(2); //获取yolo配置
    string modelWeights = parser.get<string>(3);       //获取yolo目标检测权重

    // Load names of classes
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line))
        classes.push_back(line);

    // 加载神经网络
    Net net = readNetFromDarknet(modelConfiguration, modelWeights); //将权重导入darknet模型
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    //获取视频帧，对帧进行处理
    int bytes = 0;
    for (;;)
    {

        double t0 = getTickCount(); //获取刚进入此段程序的时钟周期数

        char socketData[imgSize];
        for (int i = 0; i < imgSize; i += bytes)
        {
            if ((bytes = recv(client_socket, socketData + i, imgSize - i, 0)) == -1)
            {
                cout << "!Fault" << endl;
                exit(-1);
            }
        }
        // change the last loop to below statement
        Mat frame(Size(640, 360), CV_8UC3, socketData); //根据摄像头大小自行修改
        //Mat frame;
        //cap >> frame; //获取新的帧图像
        if (frame.empty())
        {
            cout << "couldn't give frame" << endl;
            waitKey(0);
            break;
        }

        //道路检测，并返回当前行进在道路中心点左边还是右边。
        controlflag = directionControl(frame);
        // Create a 4D blob from a frame.
        Mat blob;
        blobFromImage(frame, blob, 1 / 255.0, Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);

        //Sets the input to the network
        net.setInput(blob);

        // Runs the forward pass to get output of the output layers
        vector<Mat> outs;
        net.forward(outs, getOutputsNames(net));

        // Remove the bounding boxes with low confidence
        postprocess(frame, outs);
        imshow("output", frame);
        //发送控制信号
        char controlFlag[1024] = {(char)('0' + stopflag), (char)('0' + controlflag)};
        send(client_socket, controlFlag, 1024, 0);

        // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;           //获取一毫秒的时钟周期数
        double t1 = (getTickCount() - t0) / freq;          //获取处理一帧图像的时间
        double t = net.getPerfProfile(layersTimes) / freq; //获取检测一帧图片的时间
        printf(" 处理帧的时间: %.2f ms stopflag:%d  controlflag:%d \n", t1, stopflag, controlflag);

        //imwrite("result.jpg", frame);

        if (waitKey(1) >= 0) //按下任意键1s退出
            break;
    }

    shutdown(client_socket, SHUT_RDWR);
    return 0;
}
