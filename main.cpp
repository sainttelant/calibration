

#include "CalibrationTool.hpp"
using namespace UcitCalibrate;

int main()
{
	////// 首先通过标定板的图像像素坐标以及对应的世界坐标，通过PnP求解相机的R&T//////
	// 准备的是图像上的像素点
	// 创建输出参数文件，ios：：trunc含义是有文件先删除
	ofstream outfile("F:\\test\\calibration\\roadsideCalibration\\calibrate.txt", ios::trunc);

	

	CalibrationTool  m_Calibrations;
	CalibrationTool  m_WholeCalibrations;
	// 设置挑选点
	vector<unsigned int> pickPointindex{1,3,7,9};
	bool rasac = false;


	// dont change wholeindex
	vector<unsigned int> wholeindex{ 1,2,3,4,5,6,7,8,9 };

	// Set pi
	m_Calibrations.SetPi(CV_PI);
	vector<Point2f> boxPoints;
	std::map<int, Point2f> mp_images;
	Point2f p1(1322.0f, 750.0f);
	Point2f p2(1484.0f, 756.0f);
	Point2f p3(1642.0f, 764.0f);
	Point2f p4(1269.0f, 608.0f);
	Point2f p5(1369.0f, 613.0f);
	Point2f p6(1465.0f, 616.0f);
	Point2f p7(1247.0f, 542.0f);
	Point2f p8(1315.0f, 546.0f);
	Point2f p9(1383.0f, 549.0f);

	boxPoints.push_back(p1);
	boxPoints.push_back(p2);
	boxPoints.push_back(p3);
	boxPoints.push_back(p4);
	boxPoints.push_back(p5);
	boxPoints.push_back(p6);
	boxPoints.push_back(p7);
	boxPoints.push_back(p8);
	boxPoints.push_back(p9);

	mp_images[1] = p1;
	mp_images[2] = p2;
	mp_images[3] = p3;
	mp_images[4] = p4;
	mp_images[5] = p5;
	mp_images[6] = p6;
	mp_images[7] = p7;
	mp_images[8] = p8;
	mp_images[9] = p9;


	m_Calibrations.PickImagePixelPoints4PnPsolve(pickPointindex, mp_images);
	m_WholeCalibrations.PickImagePixelPoints4PnPsolve(wholeindex, mp_images);

	// Step one Loading image
	Mat sourceImage = imread("2.BMP");
    // 写字
    
    char textbuf[256];

	// draw 原始的取点
	for (int i = 0; i < boxPoints.size(); ++i)
	{
		circle(sourceImage, boxPoints[i], 8, Scalar(100, 255, 0), -1, LINE_AA);
        int text_x = (int)(boxPoints[i].x - 30);
        int text_y = (int)(boxPoints[i].y - 10);
        sprintf(textbuf, "RPoint%d",i);
        //putText(sourceImage, textbuf, cv::Point(text_x, text_y), FONT_HERSHEY_COMPLEX,3, Scalar(0,0,255));
	}



	namedWindow("raw image", WINDOW_NORMAL);
	
    // Step two Calculate the GPS 2 World
	// prepare GPS raw points
	double Gpslong1 = 121.30774132;
	double Gpslong2 = 121.30775749;
	double Gpslong3 = 121.30777151;
	double Gpslong4 = 121.30804771;
	double Gpslong5 = 121.30806281;
	double Gpslong6 = 121.30807791;
	double Gpslong7 = 121.30838804;
	double Gpslong8 = 121.30839387;
	double Gpslong9 = 121.30841134;

	double Gpsla1 = 31.19691151;
	double Gpsla2 = 31.19688899;
	double Gpsla3 = 31.19686509;
	double Gpsla4 = 31.19705386;
	double Gpsla5 = 31.19703155;
	double Gpsla6 = 31.19700724;
	double Gpsla7 = 31.19728386;
	double Gpsla8 = 31.19718445;
	double Gpsla9 = 31.19716105;

	std::map<int, double> mp_Gpslong, mp_Gpslat;
	mp_Gpslong[1] = Gpslong1;
	mp_Gpslong[2] = Gpslong2;
	mp_Gpslong[3] = Gpslong3;
	mp_Gpslong[4] = Gpslong4;
	mp_Gpslong[5] = Gpslong5;
	mp_Gpslong[6] = Gpslong6;
	mp_Gpslong[7] = Gpslong7;
	mp_Gpslong[8] = Gpslong8;
	mp_Gpslong[9] = Gpslong9;

	mp_Gpslat[1] = Gpsla1;
	mp_Gpslat[2] = Gpsla2;
	mp_Gpslat[3] = Gpsla3;
	mp_Gpslat[4] = Gpsla4;
	mp_Gpslat[5] = Gpsla5;
	mp_Gpslat[6] = Gpsla6;
	mp_Gpslat[7] = Gpsla7;
	mp_Gpslat[8] = Gpsla8;
	mp_Gpslat[9] = Gpsla9;


	
	
	// pick points for calibration
	m_Calibrations.PickRawGPSPoints4VectorPairs(pickPointindex, mp_Gpslong, mp_Gpslat);
	m_WholeCalibrations.PickRawGPSPoints4VectorPairs(wholeindex, mp_Gpslong, mp_Gpslat);

	// gps convert to the world coordination
	m_Calibrations.Gps2WorldCoord(m_Calibrations.gps_longPick, m_Calibrations.gps_latiPick);
	m_WholeCalibrations.Gps2WorldCoord(m_WholeCalibrations.gps_longPick, m_WholeCalibrations.gps_latiPick);
	m_WholeCalibrations.SetWorldBoxPoints();

	
	m_Calibrations.SetWorldBoxPoints();


	//Setting box corners in real world//////
	vector<Point3d> worldBoxPoints;  //存入世界坐标
	vector<Point3d> measurementPoint;
	worldBoxPoints = m_Calibrations.GetWorldBoxPoints();

	// 计算雷达的rt矩阵
	// 用于反算雷达是否标定准确,以下的点是实测距离值，单位为米
	vector<Point3d> srcPoints;

	std::map<int, Point3d> map_Measures;

	Point3d p31(-2.586f, 48.8f, 0.0f);
	Point3d p32(-5.518f, 48.8f, 0.0f);
	Point3d p33(-2.959f, 48.8f, 0.0f);
	Point3d p34(-2.583f, 79.2f, 0.0f);
	Point3d p35(-5.550f, 79.2f, 0.0f);
	Point3d p36(-3.030f, 79.2f, 0.0f);
	Point3d p37(-2.587f, 114.0f, 0.0f);
	Point3d p38(-5.545f, 114.0f, 0.0f);
	Point3d p39(-3.000f, 114.0f, 0.0f);
	map_Measures[1] = p31;
	map_Measures[2] = p32;
	map_Measures[3] = p33;
	map_Measures[4] = p34;
	map_Measures[5] = p35;
	map_Measures[6] = p36;
	map_Measures[7] = p37;
	map_Measures[8] = p38;
	map_Measures[9] = p39;

	m_Calibrations.PickMeasureMentValue4RadarRT(pickPointindex, map_Measures);



	// 计算雷达的rt矩阵

	cv::Mat RT =  m_Calibrations.Get3DR_TransMatrix(m_Calibrations.GetMeasureMentPoint(), \
		m_Calibrations.GetWorldBoxPoints());

	for (int r = 0; r < RT.rows; r++)
	{
		        for (int c = 0; c < RT.cols; c++)
		        {
		            printf("Radar's RT Matrix:%f, ", RT.at<double>(r, c));
					
		        }
		        printf("\n");
	}
	outfile << "it is a radar RT matrix first! \n" << endl;
	outfile << RT <<"\n\n"<< endl;
	

		    printf("**************************************\n");
			printf("**************************************\n");
			printf("**************************************\n");
			printf("**************************************\n");
		
	

	// camera relevant 
	// camera inside parameters
    cv::Mat cameraMatrix1 = Mat::eye(3, 3, cv::DataType<double>::type);  //相机内参矩阵
    cv::Mat distCoeffs1(5, 1, cv::DataType<double>::type);  //畸变参数
    distCoeffs1.at<double>(0, 0) = -0.37926434;//-0.425538335;//-0.449138527;
    distCoeffs1.at<double>(1, 0) = -1.35701064;//0.669067735;//-1.44777024;
    distCoeffs1.at<double>(2, 0) = 0.00229614642;//0.002579693;//-0.003672864;
    distCoeffs1.at<double>(3, 0) = 0.000660113081;//-0.0001853206;//-0.006163641;
    distCoeffs1.at<double>(4, 0) = 10.8547903;//-1.13768705;//6.11274918;

	//Taken from Mastring OpenCV d
	double fx = 4379.81913;//2763.00742;//3145.87006;//6472.05618/2;
	double fy = 4404.48029;//2762.83572;//3136.68936;//6509.35456/2;
	double cx = 1280.19495;//1240.60892;//1331.86113;//1279.92125;
	double cy = 719.938094;//661.524642;//769.252552;//719.724251;

	cameraMatrix1.at<double>(0, 0) = fx;
	cameraMatrix1.at<double>(1, 1) = fy;
	cameraMatrix1.at<double>(0, 2) = cx;
	cameraMatrix1.at<double>(1, 2) = cy;

    // 获得世界坐标系到像素坐标系的R_T矩阵
	 //////PnP solve R&T///////////////////////////////
	cv::Mat rvec1(3, 1, cv::DataType<double>::type);  //旋转向量
	cv::Mat tvec1(3, 1, cv::DataType<double>::type);  //平移向量

    //调用 pnp solve 函数

	if (rasac)
	{
		cv::solvePnPRansac(worldBoxPoints, m_Calibrations.imagePixel_pick, cameraMatrix1, distCoeffs1, rvec1, tvec1, false, SOLVEPNP_P3P);
	}
	else
	{
		if (pickPointindex.size()>4)
		{
			cv::solvePnP(worldBoxPoints, m_Calibrations.imagePixel_pick, cameraMatrix1, distCoeffs1, rvec1, tvec1, false, SOLVEPNP_ITERATIVE);
		}
		else
		{
			cv::solvePnP(worldBoxPoints, m_Calibrations.imagePixel_pick, cameraMatrix1, distCoeffs1, rvec1, tvec1, false, SOLVEPNP_P3P);
		}

		
	}

	
	
	
	//cv::solvePnP(worldBoxPoints, boxPoints, cameraMatrix1, distCoeffs1, rvec1, tvec1, false,CV_P3P);
	cv::Mat rvecM1(3, 3, cv::DataType<double>::type);  //旋转矩阵
	cv::Rodrigues(rvec1, rvecM1);





	// 反向计算到图像上的点是否正确
	cv::Mat RadarPoint = Mat::ones(4, 1, cv::DataType<double>::type);
	 Mat world_point = Mat::ones(4, 1, cv::DataType<double>::type);
	    
	   // 手动配置的雷达点

		// 雷达的原始点 unit 米
	    RadarPoint.at<double>(0, 0) = -2.58f;	//-3.0;
	    RadarPoint.at<double>(1, 0) = 114.01f;	//114.4;
	    RadarPoint.at<double>(2, 0) = 0;
		

	    world_point = RT * RadarPoint;  //OK
	    cout << "src to dst:   " << world_point << endl;



		// gps 转化成世界坐标之后的点

		Mat m_gps2world = Mat::ones(4, 1, cv::DataType<double>::type);
	
		//3D to 2D////////////////////////////
		Mat image_points = Mat::ones(3, 1, cv::DataType<double>::type);
		Mat RT_;
		hconcat(rvecM1, tvec1, RT_);
		cout << "Image RT_" << RT_ << endl;
		outfile << "it is a Image RT matrix then! \n\n" << endl;
		outfile << RT_ << endl;
		cameraMatrix1.at<double>(0, 0) = fx;
		cameraMatrix1.at<double>(1, 1) = fy;
		cameraMatrix1.at<double>(0, 2) = cx;
		cameraMatrix1.at<double>(1, 2) = cy;

		
	

		// 反推九个点投影到pixel坐标
		vector<Point3d>::iterator iter = m_WholeCalibrations.m_worldBoxPoints.begin();
		for (; iter!= m_WholeCalibrations.m_worldBoxPoints.end(); iter++)
		{
			m_gps2world.at<double>(0, 0) = iter->x;
			m_gps2world.at<double>(1, 0) = iter->y;
			m_gps2world.at<double>(2, 0) = 0;
			image_points = cameraMatrix1 * RT_ * m_gps2world;
			Mat D_Points = Mat::ones(3, 1, cv::DataType<double>::type);
			D_Points.at<double>(0, 0) = image_points.at<double>(0, 0) / image_points.at<double>(2, 0);
			D_Points.at<double>(1, 0) = image_points.at<double>(1, 0) / image_points.at<double>(2, 0);

			cout << "3D to 2D:   " << D_Points << endl;

			Point2f raderpixelPoints;
			raderpixelPoints.x = D_Points.at<double>(0, 0);
			raderpixelPoints.y = D_Points.at<double>(1, 0);
			std::string raders = "radarPoints";
			circle(sourceImage, raderpixelPoints, 15, Scalar(0, 0, 255), -1, LINE_AA);
		}

		outfile.close();
	imshow("raw image", sourceImage);
	waitKey(0);
    system("PAUSE");
    return 0;
}

