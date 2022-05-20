#include <QCoreApplication>
/*
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <math.h>
#include <iostream>
#include <fstream>
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
using namespace cv;
using namespace std;


//œ«¿ÕŒäµãÈÆZÖáÐý×ª
//ÊäÈë²ÎÊý x yÎª¿ÕŒäµãÔ­ÊŒx y×ø±ê
//thetazÎª¿ÕŒäµãÈÆZÖáÐý×ª¶àÉÙ¶È£¬œÇ¶ÈÖÆ·¶Î§ÔÚ-180µœ180
//outx outyÎªÐý×ªºóµÄœá¹û×ø±ê
void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//œ«±äÁ¿¿œ±ŽÒ»ŽÎ£¬±£Ö€&x == &outxÕâÖÖÇé¿öÏÂÒ²ÄÜŒÆËãÕýÈ·
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}

//œ«¿ÕŒäµãÈÆYÖáÐý×ª
//ÊäÈë²ÎÊý x zÎª¿ÕŒäµãÔ­ÊŒx z×ø±ê
//thetayÎª¿ÕŒäµãÈÆYÖáÐý×ª¶àÉÙ¶È£¬œÇ¶ÈÖÆ·¶Î§ÔÚ-180µœ180
//outx outzÎªÐý×ªºóµÄœá¹û×ø±ê
void codeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}

//œ«¿ÕŒäµãÈÆXÖáÐý×ª
//ÊäÈë²ÎÊý y zÎª¿ÕŒäµãÔ­ÊŒy z×ø±ê
//thetaxÎª¿ÕŒäµãÈÆXÖáÐý×ª¶àÉÙ¶È£¬œÇ¶ÈÖÆ£¬·¶Î§ÔÚ-180µœ180
//outy outzÎªÐý×ªºóµÄœá¹û×ø±ê
void codeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;//œ«±äÁ¿¿œ±ŽÒ»ŽÎ£¬±£Ö€&y == &yÕâÖÖÇé¿öÏÂÒ²ÄÜŒÆËãÕýÈ·
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}


//µãÈÆÈÎÒâÏòÁ¿Ðý×ª£¬ÓÒÊÖÏµ
//ÊäÈë²ÎÊýold_x£¬old_y£¬old_zÎªÐý×ªÇ°¿ÕŒäµãµÄ×ø±ê
//vx£¬vy£¬vzÎªÐý×ªÖáÏòÁ¿
//thetaÎªÐý×ªœÇ¶ÈœÇ¶ÈÖÆ£¬·¶Î§ÔÚ-180µœ180
//·µ»ØÖµÎªÐý×ªºó×ø±êµã
cv::Point3f RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
{
    double r = theta * CV_PI / 180;
    double c = cos(r);
    double s = sin(r);
    double new_x = (vx*vx*(1 - c) + c) * old_x + (vx*vy*(1 - c) - vz*s) * old_y + (vx*vz*(1 - c) + vy*s) * old_z;
    double new_y = (vy*vx*(1 - c) + vz*s) * old_x + (vy*vy*(1 - c) + c) * old_y + (vy*vz*(1 - c) - vx*s) * old_z;
    double new_z = (vx*vz*(1 - c) - vy*s) * old_x + (vy*vz*(1 - c) + vx*s) * old_y + (vz*vz*(1 - c) + c) * old_z;
    return cv::Point3f(new_x, new_y, new_z);
}


/************radar to world calc**************************************************************************/
cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3f>& srcPoints, const std::vector<cv::Point3f>&  dstPoints)
{
    double srcSumX = 0.0f;
    double srcSumY = 0.0f;
    double srcSumZ = 0.0f;

    double dstSumX = 0.0f;
    double dstSumY = 0.0f;
    double dstSumZ = 0.0f;

    //至少三组点
    if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
    {
        return cv::Mat();
    }

    int pointsNum = srcPoints.size();
    for (int i = 0; i < pointsNum; ++i)
    {
        srcSumX += srcPoints[i].x;
        srcSumY += srcPoints[i].y;
        srcSumZ += srcPoints[i].z;Mat RT_ = (cv::Mat_<double>(3, 4) <<
                                             0.2594418744433779, 0.96554671632539, -0.02023492966539137, -0.7979452296853149,
                                              0.1311192655904193, -0.05597456802002987, -0.9897851210874005, 6.866882835016548,
                                              -0.956816414980604, 0.2541385179940812, -0.1411239231861011, -3.072334460859187
                                        );
                                  cout<<"RT_"<<RT_<<endl;

        dstSumX += dstPoints[i].x;
        dstSumY += dstPoints[i].y;
        dstSumZ += dstPoints[i].z;
    }

    cv::Point3d centerSrc, centerDst;

    centerSrc.x = double(srcSumX / pointsNum);
    centerSrc.y = double(srcSumY / pointsNum);
    centerSrc.z = double(srcSumZ / pointsNum);

    centerDst.x = double(dstSumX / pointsNum);
    centerDst.y = double(dstSumY / pointsNum);
    centerDst.z = double(dstSumZ / pointsNum);

    //Mat::Mat(int rows, int cols, int type)
    cv::Mat srcMat(3, pointsNum, CV_64FC1);
    cv::Mat dstMat(3, pointsNum, CV_64FC1);
    //---Modify
    for (int i = 0; i < pointsNum; ++i)//N组点
    {
        //三行
        srcMat.at<double>(0, i) = srcPoints[i].x - centerSrc.x;
        srcMat.at<double>(1, i) = srcPoints[i].y - centerSrc.y;
        srcMat.at<double>(2, i) = srcPoints[i].z - centerSrc.z;

        dstMat.at<double>(0, i) = dstPoints[i].x - centerDst.x;
        dstMat.at<double>(1, i) = dstPoints[i].y - centerDst.y;
        dstMat.at<double>(2, i) = dstPoints[i].z - centerDst.z;

    }

    cv::Mat matS = srcMat * dstMat.t();

    cv::Mat matU, matW, matV;
    cv::SVDecomp(matS, matW, matU, matV);

    cv::Mat matTemp = matU * matV;
    double det = cv::determinant(matTemp);//行列式的值

    double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
    cv::Mat matM(3, 3, CV_64FC1, datM);

    cv::Mat matR = matV.t() * matM * matU.t();

    double* datR = (double*)(matR.data);
    double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
    double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
    double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);

    #if 1
    //生成RT齐次矩阵(4*4)
    cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
        matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
        matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
        matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2),delta_Z,
        0, 0, 0, 1
        );

    return R_T;
    #endif
}
/*************************************************************************************************************/
void radar2pixel(int num,float *rx,float *ry,int *px,int *py);
int main()
{
    
    int cnt = 8;
    float rx[cnt] = {-3.0,-1.4,-3.6,-2.0,-4.6,-2.8,-5.6,-3.8}; //radar dx
    float ry[cnt] = {29.6,29.6,32.8,34.4,42.4,42.4,48.8,48.8}; //radar dy
    int px[cnt] = {1569,1414,1578,1443,1578,1463,1592,1492};   //image pixel x
    int py[cnt] = { 958,979,852,854,706,706,632,634};					//image pixel x
    radar2pixel(8,rx,ry,px,py);
    return 0;
}
void radar2pixel(int num,float *rx,float *ry,int *px,int *py)
{
    Mat sourceImage = imread("/home/wang/qt_project/cam_pose_estimation/P7.jpg");
    //radar to world/////////////// 1,2,5
    Mat RT_Radar2World = (cv::Mat_<double>(4, 4) <<
                          -0.239286, -0.969987, -0.043209, 0.650732,
                          -0.970610, 0.237791, 0.037029, 1.137731,
                          -0.025642, 0.050800, -0.998380, -1.318018,
                          0.000000, 0.000000, 0.000000, 1.000000
          );
       

    // 1, 2, 5, 6
    Mat RT_ = (cv::Mat_<double>(3, 4) <<
               0.2594418744433779, 0.96554671632539, -0.02023492966539137, -0.7979452296853149,
                0.1311192655904193, -0.05597456802002987, -0.9897851210874005, 6.866882835016548,
                -0.956816414980604, 0.2541385179940812, -0.1411239231861011, -3.072334460859187
          );
    cout<<"RT_"<<RT_<<endl;


    cv::Mat RadarPoint=Mat::ones(4,1,cv::DataType<double>::type);
    Mat world_point=Mat::ones(4,1,cv::DataType<double>::type);

    for(int i= 0;i<num;i++)  {
    RadarPoint.at<double>(0,0)=rx[i];//-5.6;
    RadarPoint.at<double>(1,0)=ry[i];//48.8;
    RadarPoint.at<double>(2,0)=0;

    world_point = RT_Radar2World*RadarPoint;  //OK
    cout<<"src to dst:   "<<world_point<<endl;

    ///3D to 2D////////////////////////////
    Mat worldPoints=Mat::ones(4,1,cv::DataType<double>::type);
    worldPoints = world_point.clone();
    cout<<"world Points :  "<<worldPoints<<endl;
    Mat image_points=Mat::ones(3,1,cv::DataType<double>::type);

    cv::Mat cameraMatrix1=Mat::eye(3, 3, cv::DataType<double>::type);  //相机内参矩阵
    cv::Mat distCoeffs1(5, 1, cv::DataType<double>::type);  //畸变参数

    ///*
    distCoeffs1.at<double>(0,0) = -0.60016012;//-0.37926434;//-0.425538335;//-0.449138527;
    distCoeffs1.at<double>(1,0) = 0.80604549;//-1.35701064;//0.669067735;//-1.44777024;
    distCoeffs1.at<double>(2,0) = 0.01215003;//0.00229614642;//0.002579693;//-0.003672864;
    distCoeffs1.at<double>(3,0) = -0.00146187;//0.000660113081;//-0.0001853206;//-0.006163641;
    distCoeffs1.at<double>(4,0) = -0.65938132;//10.8547903;//-1.13768705;//6.11274918;
    //*/

    //Taken from Mastring OpenCV d
    double fx = 2562.5455;//4379.81913;//2763.00742;//3145.87006;//6472.05618/2;
    double fy = 2566.81406;//4404.48029;//2762.83572;//3136.68936;//6509.35456/2;
    double cx = 1298.28733;//1280.19495;//1240.60892;//1331.86113;//1279.92125;
    double cy = 709;//719.938094;//661.524642;//769.252552;//719.724251;


    //////camera  intristic///////////////////////////////

    cameraMatrix1.at<double>(0, 0) = fx;
    cameraMatrix1.at<double>(1, 1) = fy;
    cameraMatrix1.at<double>(0, 2) = cx;
    cameraMatrix1.at<double>(1, 2) = cy;

    image_points=cameraMatrix1*RT_*worldPoints;
    Mat D_Points=Mat::ones(3,1,cv::DataType<double>::type);
    D_Points.at<double>(0,0)=image_points.at<double>(0,0)/image_points.at<double>(2,0);
    D_Points.at<double>(1,0)=image_points.at<double>(1,0)/image_points.at<double>(2,0);
    //cv::projectPoints(worldPoints, rvec1, tvec1, cameraMatrix1, distCoeffs1, imagePoints);
    cout<<"3D to 2D:   "<<D_Points<<endl;
    Point2f point_dst;
    point_dst.x = D_Points.at<double>(0,0);
    point_dst.y = D_Points.at<double>(1,0);
    circle(sourceImage, point_dst, 3, Scalar(0, 0, 255), -1, 8);
    D_Points.at<double>(0,0)=image_points.at<double>(0,0)/image_points.at<double>(2,0);
    D_Points.at<double>(1,0)=image_points.at<double>(1,0)/image_points.at<double>(2,0);
    //cv::projectPoints(worldPoints, rvec1, tvec1, cameraMatrix1, distCoeffs1, imagePoints);


    point_dst.x = px[i];
    point_dst.y = py[i];
    circle(sourceImage, point_dst, 3, Scalar(0, 255, 255), -1, 8);
    }
    namedWindow("output",0);
    resizeWindow("output", 1920, 1080);
    imshow("output",sourceImage);
    waitKey(0);
}
