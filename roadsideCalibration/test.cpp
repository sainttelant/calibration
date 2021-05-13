
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
#include <math.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;



void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//??±?????±???????±??�?x == &outx?????é???????????????·
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}

void codeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}


void codeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;//??±?????±???????±??�?y == &y?????é???????????????·
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}



cv::Point3f RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
{
    double r = theta * CV_PI / 180;
    double c = cos(r);
    double s = sin(r);
    double new_x = (vx * vx * (1 - c) + c) * old_x + (vx * vy * (1 - c) - vz * s) * old_y + (vx * vz * (1 - c) + vy * s) * old_z;
    double new_y = (vy * vx * (1 - c) + vz * s) * old_x + (vy * vy * (1 - c) + c) * old_y + (vy * vz * (1 - c) - vx * s) * old_z;
    double new_z = (vx * vz * (1 - c) - vy * s) * old_x + (vy * vz * (1 - c) + vx * s) * old_y + (vz * vz * (1 - c) + c) * old_z;
    return cv::Point3f(new_x, new_y, new_z);
}


/************radar to world calc**************************************************************************/
cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3f>& srcPoints, const std::vector<cv::Point3f>& dstPoints)
{
    double srcSumX = 0.0f;
    double srcSumY = 0.0f;
    double srcSumZ = 0.0f;

    double dstSumX = 0.0f;
    double dstSumY = 0.0f;
    double dstSumZ = 0.0f;

    //至少三组�?
    if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
    {
        return cv::Mat();
    }

    int pointsNum = srcPoints.size();
    for (int i = 0; i < pointsNum; ++i)
    {
        srcSumX += srcPoints[i].x;
        srcSumY += srcPoints[i].y;
        srcSumZ += srcPoints[i].z;

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
    double det = cv::determinant(matTemp);//行列式的�?

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
        matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
        0, 0, 0, 1
        );

    return R_T;
#endif
}
/*************************************************************************************************************/
int main()
{
#if 1
    /*********************radar 2  wcs calc******************************/
    std::vector<cv::Point3f> srcPoints;
    std::vector<cv::Point3f>  dstPoints;

    std::vector<cv::Point3f> srcPointsTest;
    std::vector<cv::Point3f>  dstPointsTest;

    //模拟绕Z轴转90度，再X轴平�?0.
    //取三组点
    /*
    float NN = 100;
    srcPoints.push_back(cv::Point3f(NN, 0, 0));
    dstPoints.push_back(cv::Point3f(50+0, NN, 0));

    srcPoints.push_back(cv::Point3f(0, NN, 0));
    dstPoints.push_back(cv::Point3f(50 -NN, 0, 0));

    srcPoints.push_back(cv::Point3f(0, 0, NN));
    dstPoints.push_back(cv::Point3f(50+0, 0, NN));
   */
   //srcPoints.push_back(cv::Point3f(1.8, 42.4, 0));
    srcPoints.push_back(cv::Point3f(-2.4, 42.4, 0));
    dstPoints.push_back(cv::Point3f(41.410419, 19.746567, 0));

    //srcPoints.push_back(cv::Point3f(-4.6, 42.4, 0));
    srcPoints.push_back(cv::Point3f(-9.0, 42.40, 0));
    dstPoints.push_back(cv::Point3f(44.316414, 13.801364, 0));

    //srcPoints.push_back(cv::Point3f(5.8, 112.8, 0));
    srcPoints.push_back(cv::Point3f(-6.2, 112.4, 0));
    dstPoints.push_back(cv::Point3f(103.162804, 50.109566, 0));

    cv::Mat RT = Get3DR_TransMatrix(srcPoints, dstPoints);
    for (int r = 0; r < RT.rows; r++)
    {
        for (int c = 0; c < RT.cols; c++)
        {
            printf("%f, ", RT.at<double>(r, c));
        }
        printf("\n");
    }
    printf("**************************************\n");

    cv::Mat worldPoints1 = Mat::ones(4, 1, cv::DataType<double>::type);
    Mat image_points1 = Mat::ones(4, 1, cv::DataType<double>::type);
    worldPoints1.at<double>(0, 0) = -10;//-3.4;
    worldPoints1.at<double>(1, 0) = 186;//42.4;
    worldPoints1.at<double>(2, 0) = 0;
    srcPointsTest.push_back(cv::Point3f(0, 0, 0));

    cv::Mat srcMat(4, 1, CV_64FC1);
    cv::Mat dstMat(4, 1, CV_64FC1);
    // srcMat.
    image_points1 = RT * worldPoints1;  //OK
    cout << "src to dst:   " << image_points1 << endl;
    dstMat = RT * srcMat;
    cout << "src to dst:   " << dstMat << endl;
#endif
    /********************************************************************/
#if 1
  ////// 首先通过标定板的图像像素坐标以及对应的世界坐标，通过PnP求解相机的R&T//////
    Point2f point;
    vector<Point2f> boxPoints; //存入像素坐标
    // Loading image
    Mat sourceImage = imread("/home/wang/qt_project/cam_pose_estimation/2.jpg");
    //namedWindow("Source", 1);
    ///// Setting box corners in image
    //////one Point////


    //point = Point2f((float) 878, (float) 660); //640X480
    //point = Point2f((float) 954, (float) 750); //640X480
    point = Point2f((float)472, (float)893); //2K 2020.1.25 cam biaoding
    boxPoints.push_back(point);
    circle(sourceImage, boxPoints[0], 3, Scalar(0, 255, 0), -1, 8);

    ////two Point////
    //point = Point2f((float) 1245, (float) 675); //640X480
    //point = Point2f((float) 1338, (float) 774); //640X480
    point = Point2f((float)1338, (float)774); //
    boxPoints.push_back(point);
    circle(sourceImage, boxPoints[1], 3, Scalar(0, 255, 0), -1, 8);

    ////three Point////
    //point = Point2f((float) 874, (float) 444); //640X480
    //point = Point2f((float) 978, (float) 538); //640X480
    point = Point2f((float)810, (float)464); //
    boxPoints.push_back(point);
    circle(sourceImage, boxPoints[2], 3, Scalar(0, 255, 0), -1, 8);

    ////four Point////
    //point = Point2f((float) 948, (float) 442); //640X480
    //point = Point2f((float) 1064, (float) 542); //640X480
    point = Point2f((float)996, (float)477);
    boxPoints.push_back(point);
    circle(sourceImage, boxPoints[3], 3, Scalar(0, 255, 0), -1, 8);
    //////////Setting box corners in real world////////////////////
    vector<Point3f> worldBoxPoints;  //存入世界坐标
    Point3f tmpPoint;

    //tmpPoint = Point3f((float) 41.410419, (float) 19.746567, (float) 0);  //2020.04.20 Ucit-2119
    //worldBoxPoints.push_back(tmpPoint);
    //tmpPoint = Point3f((float) 44.316414, (float) 13.801364 , (float) 0);
    //worldBoxPoints.push_back(tmpPoint);
    //tmpPoint = Point3f((float) 103.162804 , (float) 50.109566, (float) 0);
    //worldBoxPoints.push_back(tmpPoint);
    //tmpPoint = Point3f((float) 104.615807, (float)  47.349293, (float) 0);
    //worldBoxPoints.push_back(tmpPoint);

    tmpPoint = Point3f((float)25.427452, (float)19.109581, (float)0);  ////2K 2020.1.25 cam biaoding
    worldBoxPoints.push_back(tmpPoint);
    tmpPoint = Point3f((float)44.316414, (float)13.801364, (float)0);
    worldBoxPoints.push_back(tmpPoint);
    tmpPoint = Point3f((float)179.445160, (float)100.431465, (float)0);
    worldBoxPoints.push_back(tmpPoint);
    tmpPoint = Point3f((float)179.445160, (float)85.568459, (float)0);
    worldBoxPoints.push_back(tmpPoint);

    //////camera  intristic///////////////////////////////
    cv::Mat cameraMatrix1 = Mat::eye(3, 3, cv::DataType<double>::type);  //相机内参矩阵
    cv::Mat distCoeffs1(5, 1, cv::DataType<double>::type);  //畸变参数

    ///*
    distCoeffs1.at<double>(0, 0) = -0.37926434;//-0.425538335;//-0.449138527;
    distCoeffs1.at<double>(1, 0) = -1.35701064;//0.669067735;//-1.44777024;
    distCoeffs1.at<double>(2, 0) = 0.00229614642;//0.002579693;//-0.003672864;
    distCoeffs1.at<double>(3, 0) = 0.000660113081;//-0.0001853206;//-0.006163641;
    distCoeffs1.at<double>(4, 0) = 10.8547903;//-1.13768705;//6.11274918;
    //*/

    //Taken from Mastring OpenCV d
    double fx = 4379.81913;//2763.00742;//3145.87006;//6472.05618/2;
    double fy = 4404.48029;//2762.83572;//3136.68936;//6509.35456/2;
    double cx = 1280.19495;//1240.60892;//1331.86113;//1279.92125;
    double cy = 719.938094;//661.524642;//769.252552;//719.724251;

    cameraMatrix1.at<double>(0, 0) = fx;
    cameraMatrix1.at<double>(1, 1) = fy;
    cameraMatrix1.at<double>(0, 2) = cx;
    cameraMatrix1.at<double>(1, 2) = cy;

    //////PnP solve R&T///////////////////////////////
    cv::Mat rvec1(3, 1, cv::DataType<double>::type);  //旋转向量
    cv::Mat tvec1(3, 1, cv::DataType<double>::type);  //平移向量
    cv::solvePnP(worldBoxPoints, boxPoints, cameraMatrix1, distCoeffs1, rvec1, tvec1, false, SOLVEPNP_ITERATIVE);
    //cv::solvePnP(worldBoxPoints, boxPoints, cameraMatrix1, distCoeffs1, rvec1, tvec1, false,CV_P3P);
    cv::Mat rvecM1(3, 3, cv::DataType<double>::type);  //旋转矩阵
    cv::Rodrigues(rvec1, rvecM1);


    double thetaZ = atan2(rvecM1.at<double>(1, 0), rvecM1.at<double>(0, 0)) / CV_PI * 180;
    double thetaY = atan2(-1 * rvecM1.at<double>(2, 0), sqrt(rvecM1.at<double>(2, 1) * rvecM1.at<double>(2, 1)
        + rvecM1.at<double>(2, 2) * rvecM1.at<double>(2, 2))) / CV_PI * 180;
    double thetaX = atan2(rvecM1.at<double>(2, 1), rvecM1.at<double>(2, 2)) / CV_PI * 180;
    cout << "theta x  " << thetaX << endl << "theta Y: " << thetaY << endl << "theta Z: " << thetaZ << endl;
    ///////////////////////////////////////////////////////////////////////////////////////////////

    double rm[9];
    cv::Mat rotM(3, 3, CV_64FC1, rm);
    Rodrigues(rvec1, rotM);
    double r11 = rotM.ptr<double>(0)[0];
    double r12 = rotM.ptr<double>(0)[1];
    double r13 = rotM.ptr<double>(0)[2];
    double r21 = rotM.ptr<double>(1)[0];
    double r22 = rotM.ptr<double>(1)[1];
    double r23 = rotM.ptr<double>(1)[2];
    double r31 = rotM.ptr<double>(2)[0];
    double r32 = rotM.ptr<double>(2)[1];
    double r33 = rotM.ptr<double>(2)[2];

    /*************************************此处计算出相机的旋转�?*********************************************/
    //计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系�?
    //旋转顺序为z、y、x
    //原理见帖子：
    double thetaz = atan2(r21, r11) / CV_PI * 180;
    double thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
    double thetax = atan2(r32, r33) / CV_PI * 180;

    std::cout << "相机的三轴旋转角�? << -1 * thetax << ", " << -1 * thetay << ", " << -1 * thetaz << std::endl;

        double tx = tvec1.ptr<double>(0)[0];
    double ty = tvec1.ptr<double>(0)[1];
    double tz = tvec1.ptr<double>(0)[2];

    //x y z 为唯一向量在相机原始坐标系下的向量�?
    //也就是向量OcOw在相机坐标系下的�?
    double x = tx, y = ty, z = tz;

    //进行三次反向旋转
    codeRotateByZ(x, y, -1 * thetaZ, x, y);
    codeRotateByY(x, z, -1 * thetaY, x, z);
    codeRotateByX(y, z, -1 * thetaX, y, z);


    //获得相机在世界坐标系下的位置坐标
    //即向量OcOw在世界坐标系下的�?
    double Cx = x * -1;
    double Cy = y * -1;
    double Cz = z * -1;

    //ofstream fout2("D:\\pnp_t.txt");
    //fout2 << Cx << endl << Cy << endl << Cz << endl;
    cout << "相机的世界坐标：" << Cx << ", " << Cy << ", " << Cz << endl;
    //fout2.close();

///////////////根据公式求Zc，即s////////////////////////
    cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
    cv::Mat tempMat, tempMat2;
    //输入一�?D坐标点，便可以求出相应的s
    //imagePoint.at<double>(0,0)=558;
    //imagePoint.at<double>(1,0)=259;
    imagePoint.at<double>(0, 0) = 971;//1318;
    imagePoint.at<double>(1, 0) = 794;//695;
    double zConst = 0;//实际坐标系的距离
    //计算参数s
    double s;
    tempMat = rvecM1.inv() * cameraMatrix1.inv() * imagePoint;
    tempMat2 = rvecM1.inv() * tvec1;
    s = zConst + tempMat2.at<double>(2, 0);
    s /= tempMat.at<double>(2, 0);
    cout << "s : " << s << endl;
#endif
#if 1
    //radar to world///////////////
    Mat RT_Radar2World = (cv::Mat_<double>(4, 4) <<
        //-0.549571, -0.835447, 0.000000, -0.115109,
        //-0.835447, 0.549571, 0.000000, -0.048966,
        //0.000000, 0.000000, -1.000000, -0.000000,
        //0.000000, 0.000000, 0.000000, 1.000000
              //-0.395082, 0.918646, 0.000000, 3.340091,
              //0.918646, 0.395082, 0.000000, 1.155175,
              //0.000000, 0.000000, -1.000000, -0.000000,
              //0.000000, 0.000000, 0.000000, 1.000000
        -0.490950, 0.871187, 0.000000, 1.437774,
        0.871187, 0.490950, 0.000000, -0.358310,
        0.000000, 0.000000, -1.000000, -0.000000,
        0.000000, 0.000000, 0.000000, 1.000000
        );

    cv::Mat RadarPoint = Mat::ones(4, 1, cv::DataType<double>::type);
    Mat world_point = Mat::ones(4, 1, cv::DataType<double>::type);
    //RadarPoint.at<double>(0,0)=2.4;
    //RadarPoint.at<double>(1,0)=111.2;
    RadarPoint.at<double>(0, 0) = -6;//-3.0;
    RadarPoint.at<double>(1, 0) = 186;//114.4;
    RadarPoint.at<double>(2, 0) = 0;

    world_point = RT_Radar2World * RadarPoint;  //OK
    cout << "src to dst:   " << world_point << endl;

    ///3D to 2D////////////////////////////
    Mat worldPoints = Mat::ones(4, 1, cv::DataType<double>::type);
    worldPoints = world_point.clone();
    //worldPoints.at<double>(0,0)=42.863419;
    //worldPoints.at<double>(1,0)=17.198632;
    //worldPoints.at<double>(2,0)=0.4;
    cout << "world Points :  " << worldPoints << endl;
    Mat image_points = Mat::ones(3, 1, cv::DataType<double>::type);

    //Mat RT_ = (cv::Mat_<double>(3, 4) <<
    //       0.6071892363002379, 0.7923337994594953, 0.05940018144091574, 0.4744833778597741,
    //        0.1494859061488767, -0.0404914567420139, -0.9879344137106262, 3.448997902385718,
     //       -0.7803686277548468, 0.6087426321237815, -0.1430287126804526, 11.40262086503216
     //     );
    Mat RT_;
    hconcat(rvecM1, tvec1, RT_);

    cout << "RT_" << RT_ << endl;
    //////camera  intristic///////////////////////////////
    //cv::Mat cameraMatrix1=Mat::eye(3, 3, cv::DataType<double>::type);  //相机内参矩阵

    //Taken from Mastring OpenCV d
    //double fx = 4379.81913;
    //double fy = 4404.48029;
    //double cx = 1280.19495;
    //double cy = 719.938094;

    cameraMatrix1.at<double>(0, 0) = fx;
    cameraMatrix1.at<double>(1, 1) = fy;
    cameraMatrix1.at<double>(0, 2) = cx;
    cameraMatrix1.at<double>(1, 2) = cy;

    image_points = cameraMatrix1 * RT_ * worldPoints;
    Mat D_Points = Mat::ones(3, 1, cv::DataType<double>::type);
    D_Points.at<double>(0, 0) = image_points.at<double>(0, 0) / image_points.at<double>(2, 0);
    D_Points.at<double>(1, 0) = image_points.at<double>(1, 0) / image_points.at<double>(2, 0);
    //cv::projectPoints(worldPoints, rvec1, tvec1, cameraMatrix1, distCoeffs1, imagePoints);
    cout << "3D to 2D:   " << D_Points << endl;
#endif


#if 1
    //////////////////////camera_coordinates////////////////
    Mat camera_cordinates = -rvecM1.inv() * tvec1;
    /////////////////////2D to 3D///////////////////////
    cv::Mat imagePoint_your_know = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
    imagePoint_your_know.at<double>(0, 0) = 971;
    imagePoint_your_know.at<double>(1, 0) = 794;
    Mat wcPoint = rvecM1.inv() * (cameraMatrix1.inv() * s * imagePoint_your_know - tvec1);
    Point3f worldPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
    cout << "2D to 3D :" << worldPoint << endl;
    namedWindow("input", 0);
    resizeWindow("input", 640, 480);
    imshow("input", sourceImage);
#endif
    waitKey(0);
    return 0;
}