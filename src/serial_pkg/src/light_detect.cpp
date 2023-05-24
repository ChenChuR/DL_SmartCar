#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <iostream>


#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
 
using namespace std;
 
// Function headers
int processImgR(cv::Mat);
int processImgG(cv::Mat);
bool isIntersected(cv::Rect, cv::Rect);

int fmain(cv::Mat image);
 
// Global variables
bool isFirstDetectedR = true;
bool isFirstDetectedG = true;
cv::Rect* lastTrackBoxR;
cv::Rect* lastTrackBoxG;
int lastTrackNumR;
int lastTrackNumG;

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//     try
//     {
//         cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) ->image;
//         fmain(image);
//     }
//     catch(cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_brifge exception : %s", e.what());
//     }
// }

 
//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_light");

    ROS_INFO("Node Finished");

    ros::NodeHandle nh;

    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);


    ros::Publisher pub = nh.advertise<std_msgs::String>("light", 10);

    std_msgs::String msg;


    int redCount = 0;
    int greenCount = 0;
 
    cv::Mat frame;
    cv::Mat img;
    cv::Mat imgYCrCb;
    cv::Mat imgGreen;
    cv::Mat imgRed;
 
    // 亮度参数
    double a = 0.3;
    double b = (1 - a) * 125;
 
    //VideoCapture capture("C:/Users/86177/Desktop/image/123.mp4");//导入视频的路径
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cout << "Start device failed!\n" << endl;//启动设备失败！
        return -1;
    }
 
    // 帧处理
    while (1)
    {
        cap >> frame;
        //调整亮度
        frame.convertTo(img, img.type(), a, b);
 
        //转换为YCrCb颜色空间
        cvtColor(img, imgYCrCb,cv::ColorConversionCodes::COLOR_BGR2YCrCb);
 
        imgRed.create(imgYCrCb.rows, imgYCrCb.cols, CV_8UC1);
        imgGreen.create(imgYCrCb.rows, imgYCrCb.cols, CV_8UC1);
 
        //分解YCrCb的三个成分
        vector<cv::Mat> planes;
        split(imgYCrCb, planes);
        // 遍历以根据Cr分量拆分红色和绿色
        cv::MatIterator_<uchar> it_Cr = planes[1].begin<uchar>(),
                it_Cr_end = planes[1].end<uchar>();
        cv::MatIterator_<uchar> it_Red = imgRed.begin<uchar>();
        cv::MatIterator_<uchar> it_Green = imgGreen.begin<uchar>();
 
        for (; it_Cr != it_Cr_end; ++it_Cr, ++it_Red, ++it_Green)
        {
            // RED, 145<Cr<470 红色
            if (*it_Cr > 145 && *it_Cr < 470)
                *it_Red = 255;
            else
                *it_Red = 0;
 
            // GREEN 95<Cr<110 绿色
            if (*it_Cr > 95 && *it_Cr < 110)
                *it_Green = 255;
            else
                *it_Green = 0;
        }
 
        //膨胀和腐蚀
        dilate(imgRed, imgRed, cv::Mat(15, 15, CV_8UC1), cv::Point(-1, -1));
        erode(imgRed, imgRed, cv::Mat(1, 1, CV_8UC1), cv::Point(-1, -1));
        dilate(imgGreen, imgGreen, cv::Mat(15, 15, CV_8UC1), cv::Point(-1, -1));
        erode(imgGreen, imgGreen, cv::Mat(1, 1, CV_8UC1), cv::Point(-1, -1));
 
        redCount = processImgR(imgRed);
        greenCount = processImgG(imgGreen);
        cout << "red:" << redCount << ";  " << "green:" << greenCount << endl;
 
        if(redCount == 0 && greenCount == 0)
        {
            cv::putText(frame, "lights out", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 8, 8, 0);
            msg.data = "lights out";
        }else if(redCount > greenCount)
        {
            cv::putText(frame, "red light", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 8, 8, 0);
            msg.data = "red light";
        }else{
            cv::putText(frame, "green light", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 8, 8, 0);
            msg.data = "red light";
        }

        pub.publish(msg);

        ROS_INFO("light result : %s", msg.data.c_str());
 
        //imshow("video", frame);
        //imshow("Red", imgRed);
        //imshow("Green", imgGreen);
 
        // Handle with the keyboard input
        if (cv::waitKey(20) == 'q')
            break;
    }
 
    return 0;
}


// int fmain(cv::Mat image)
// {

//     std_msgs::String msg;
//     int redCount = 0;
//     int greenCount = 0;
 
//     cv::Mat frame;
//     cv::Mat img;
//     cv::Mat imgYCrCb;
//     cv::Mat imgGreen;
//     cv::Mat imgRed;
 
//     // 亮度参数
//     double a = 0.3;
//     double b = (1 - a) * 125;
 

//     frame = image;
//     //VideoCapture capture("C:/Users/86177/Desktop/image/123.mp4");//导入视频的路径
//     // cv::VideoCapture cap(0);
//     // if (!cap.isOpened())
//     // {
//     //     cout << "Start device failed!\n" << endl;//启动设备失败！
//     //     return -1;
//     // }
 
//     // 帧处理
//     while (1)
//     {
//         //cap >> frame;
//         //调整亮度
//         frame.convertTo(img, img.type(), a, b);
 
//         //转换为YCrCb颜色空间
//         cvtColor(img, imgYCrCb,cv::ColorConversionCodes::COLOR_BGR2YCrCb);
 
//         imgRed.create(imgYCrCb.rows, imgYCrCb.cols, CV_8UC1);
//         imgGreen.create(imgYCrCb.rows, imgYCrCb.cols, CV_8UC1);
 
//         //分解YCrCb的三个成分
//         vector<cv::Mat> planes;
//         split(imgYCrCb, planes);
//         // 遍历以根据Cr分量拆分红色和绿色
//         cv::MatIterator_<uchar> it_Cr = planes[1].begin<uchar>(),
//                 it_Cr_end = planes[1].end<uchar>();
//         cv::MatIterator_<uchar> it_Red = imgRed.begin<uchar>();
//         cv::MatIterator_<uchar> it_Green = imgGreen.begin<uchar>();
 
//         for (; it_Cr != it_Cr_end; ++it_Cr, ++it_Red, ++it_Green)
//         {
//             // RED, 145<Cr<470 红色
//             if (*it_Cr > 145 && *it_Cr < 470)
//                 *it_Red = 255;
//             else
//                 *it_Red = 0;
 
//             // GREEN 95<Cr<110 绿色
//             if (*it_Cr > 95 && *it_Cr < 110)
//                 *it_Green = 255;
//             else
//                 *it_Green = 0;
//         }
 
//         //膨胀和腐蚀
//         dilate(imgRed, imgRed, cv::Mat(15, 15, CV_8UC1), cv::Point(-1, -1));
//         erode(imgRed, imgRed, cv::Mat(1, 1, CV_8UC1), cv::Point(-1, -1));
//         dilate(imgGreen, imgGreen, cv::Mat(15, 15, CV_8UC1), cv::Point(-1, -1));
//         erode(imgGreen, imgGreen, cv::Mat(1, 1, CV_8UC1), cv::Point(-1, -1));
 
//         redCount = processImgR(imgRed);
//         greenCount = processImgG(imgGreen);
//         cout << "red:" << redCount << ";  " << "green:" << greenCount << endl;
 
//         if(redCount == 0 && greenCount == 0)
//         {
//             cv::putText(frame, "lights out", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 8, 8, 0);
//             msg.data = "lights out";
//         }else if(redCount > greenCount)
//         {
//             cv::putText(frame, "red light", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 8, 8, 0);
//             msg.data = "red light";
//         }else{
//             cv::putText(frame, "green light", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 8, 8, 0);
//             msg.data = "red light";
//         }

//         pub.publish(msg);

//         ROS_INFO("light result : %s", msg.data.c_str());
 
//         //imshow("video", frame);
//         //imshow("Red", imgRed);
//         //imshow("Green", imgGreen);
 
//         // Handle with the keyboard input
//         if (cv::waitKey(20) == 'q')
//             break;
//     }
 
//     return 0;
// }


 
int processImgR(cv::Mat src)
{
    cv::Mat tmp;
 
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    vector<cv::Point> hull;
 
    CvPoint2D32f tempNode;
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* pointSeq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage);
 
    cv::Rect* trackBox;
    cv::Rect* result;
    int resultNum = 0;
 
    int area = 0;
    src.copyTo(tmp);
 
    //提取轮廓
    findContours(tmp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); //用contours来存取所有轮廓
 
    if (contours.size() > 0)
    {
        trackBox = new cv::Rect[contours.size()];
        result = new cv::Rect[contours.size()];
 
        //确定要跟踪的区域
        for (int i = 0; i < contours.size(); i++)
        {
            cvClearSeq(pointSeq);
            // 获取凸包的点集
            convexHull(cv::Mat(contours[i]), hull, true);
            int hullcount = (int)hull.size();
            // 凸包的保存点
            for (int j = 0; j < hullcount - 1; j++)
            {
                tempNode.x = hull[j].x;     //凸包的集合的每个元素的X值
                tempNode.y = hull[j].y;     //凸包的集合的每个元素的Y值
                cvSeqPush(pointSeq, &tempNode);  //将凸包的点的集合传入pointSeq
            }
            cv::Mat matSeq = cv::cvarrToMat(pointSeq);
            trackBox[i] = cv::boundingRect(matSeq);  //用cvBoundingRect计算凸包的元素点的边界矩形
        }
 
        if (isFirstDetectedR)
        {
            lastTrackBoxR = new cv::Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++)
                lastTrackBoxR[i] = trackBox[i];    //存储这一帧的轮廓的边界矩形
            lastTrackNumR = contours.size();       //存储这一帧的轮廓总数
            isFirstDetectedR = false;              //以后的图片是第一帧数据
        }
        else
        {
            for (int i = 0; i < contours.size(); i++)  //扫描每一个轮廓
            {
                for (int j = 0; j < lastTrackNumR; j++)  //扫描上一帧的每一轮廓
                {
                    if (isIntersected(trackBox[i], lastTrackBoxR[j]))  //和上一帧有相交
                    {
                        result[resultNum] = trackBox[i];    //重合的矩形存入result
                        break;
                    }
                }
                resultNum++;                                //重合矩形数加1
            }
            delete[] lastTrackBoxR;                         //更新边界矩形
            lastTrackBoxR = new cv::Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++)
            {
                lastTrackBoxR[i] = trackBox[i];
            }
            lastTrackNumR = contours.size();
        }
 
        delete[] trackBox;
    }
    else
    {
        isFirstDetectedR = true;
        result = NULL;
    }
    cvReleaseMemStorage(&storage);
 
    if (result != NULL)
    {
        for (int i = 0; i < resultNum; i++)
        {
            area += result[i].area();     //计算所有红色区域
        }
    }
    delete[] result;
 
    return area;
}
 
int processImgG(cv::Mat src)
{
    cv::Mat tmp;
 
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    vector<cv::Point > hull;
 
    CvPoint2D32f tempNode;
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* pointSeq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage);
 
    cv::Rect* trackBox;
    cv::Rect* result;
    int resultNum = 0;
 
    int area = 0;
 
    src.copyTo(tmp);
    //提取轮廓
    findContours(tmp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
 
    if (contours.size() > 0)
    {
        trackBox = new cv::Rect[contours.size()];
        result = new cv::Rect[contours.size()];
 
        // 确定要跟踪的区域
        for (int i = 0; i < contours.size(); i++)
        {
            cvClearSeq(pointSeq);
            // 获取凸包的点集
            convexHull(cv::Mat(contours[i]), hull, true);
            int hullcount = (int)hull.size();
            // 保存凸包的点
            for (int j = 0; j < hullcount - 1; j++)
            {
                tempNode.x = hull[j].x;
                tempNode.y = hull[j].y;
                cvSeqPush(pointSeq, &tempNode);
            }
            cv::Mat matSeq = cv::cvarrToMat(pointSeq);
            trackBox[i] = cv::boundingRect(matSeq);
        }
 
        if (isFirstDetectedG)
        {
            lastTrackBoxG = new cv::Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++)
                lastTrackBoxG[i] = trackBox[i];
            lastTrackNumG = contours.size();
            isFirstDetectedG = false;
        }
        else
        {
            for (int i = 0; i < contours.size(); i++)
            {
                for (int j = 0; j < lastTrackNumG; j++)
                {
                    if (isIntersected(trackBox[i], lastTrackBoxG[j]))
                    {
                        result[resultNum] = trackBox[i];
                        break;
                    }
                }
                resultNum++;
            }
            delete[] lastTrackBoxG;
            lastTrackBoxG = new cv::Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++)
            {
                lastTrackBoxG[i] = trackBox[i];
            }
            lastTrackNumG = contours.size();
        }
 
        delete[] trackBox;
    }
    else
    {
        isFirstDetectedG = true;
        result = NULL;
    }
    cvReleaseMemStorage(&storage);
 
    if (result != NULL)
    {
        for (int i = 0; i < resultNum; i++)
        {
            area += result[i].area();
        }
    }
    delete[] result;
 
    return area;
}
 
//确定两个矩形区域是否相交
bool isIntersected(cv::Rect r1, cv::Rect r2)
{
    int minX = max(r1.x, r2.x);
    int minY = max(r1.y, r2.y);
    int maxX = min(r1.x + r1.width, r2.x + r2.width);
    int maxY = min(r1.y + r1.height, r2.y + r2.height);
 
    if (minX < maxX && minY < maxY)
        return true;
    else
        return false;
}