

#include "CalibrationTool.hpp"
#include <ctime>
using namespace UcitCalibrate;

int main()
{
	////// 首先通过标定板的图像像素坐标以及对应的世界坐标，通过PnP求解相机的R&T//////
	// 准备的是图像上的像素点
	// 创建输出参数文件，ios：：trunc含义是有文件先删除
	ofstream outfile("F:\\test\\calibration\\roadsideCalibration\\calibrate.txt", ios::trunc);

	// 基于当前系统的当前日期/时间
	time_t now = time(0);
	// 把 now 转换为字符串形式
	
	
	

	CalibrationTool  &m_Calibrations = CalibrationTool::getInstance();
	CalibrationTool  &m_WholeCalibrations = CalibrationTool::getInstance();
	// 设置挑选点
	vector<unsigned int> pickPointindex{1,2,3,4,5,6,7,8,9,10,11,12};
	bool rasac = false;


	// dont change wholeindex
	vector<unsigned int> wholeindex{ 1,2,3,4,5,6,7,8,9,10,11,12};

	// Set pi
	m_Calibrations.SetPi(CV_PI);
	m_Calibrations.SetRadarHeight(1.2);
	vector<Point2d> boxPoints, validPoints;
	
	std::map<int, Point2d> mp_images;
	Point2d p1(1322.0f, 750.0f);
	Point2d p2(1484.0f, 756.0f);
	Point2d p3(1642.0f, 764.0f);
	Point2d p4(1269.0f, 608.0f);
	Point2d p5(1369.0f, 613.0f);
	Point2d p6(1465.0f, 616.0f);
	Point2d p7(1247.0f, 542.0f);
	Point2d p8(1315.0f, 546.0f);
	Point2d p9(1383.0f, 549.0f);
	Point2d p10(1224.0f, 484.0f);
	Point2d p11(1263.0f, 488.0f);
	Point2d p12(1315.0f, 491.0f);
	Point2d p13(1169.0f, 481.0f);
	//Point2d p14(1270.0f, 464.0f);



	boxPoints.push_back(p1);
	boxPoints.push_back(p2);
	boxPoints.push_back(p3);
	boxPoints.push_back(p4);
	boxPoints.push_back(p5);
	boxPoints.push_back(p6);
	boxPoints.push_back(p7);
	boxPoints.push_back(p8);
	boxPoints.push_back(p9);
	boxPoints.push_back(p10);
	boxPoints.push_back(p11);
	boxPoints.push_back(p12);
	boxPoints.push_back(p13);
	//boxPoints.push_back(p14);



	mp_images[1] = p1;
	mp_images[2] = p2;
	mp_images[3] = p3;
	mp_images[4] = p4;
	mp_images[5] = p5;
	mp_images[6] = p6;
	mp_images[7] = p7;
	mp_images[8] = p8;
	mp_images[9] = p9;
	mp_images[10] = p10;
	mp_images[11] = p11;
	mp_images[12] = p12;
	mp_images[13] = p13;
	//mp_images[14] = p14;



	m_Calibrations.PickImagePixelPoints4PnPsolve(pickPointindex, mp_images);
	m_WholeCalibrations.PickImagePixelPoints4PnPsolve(wholeindex, mp_images);

	// Step one Loading image
	Mat sourceImage = imread("2.BMP");
    // 写字
    
    char textbuf[256];

	// draw 原始的取点
	for (int i = 0; i < boxPoints.size()-1; ++i)
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
	double Gpslong7 = 121.30837822;
	double Gpslong8 = 121.30839387;
	double Gpslong9 = 121.30841134;
	double Gpslong10,Gpslong11, Gpslong12,Gpslong13, Gpslong14;
	double Gpsla10, Gpsla11, Gpsla12, Gpsla13, Gpsla14;
	Gpslong10 = 121.30914588;
	Gpslong11 = 121.30914964;
	Gpslong12 = 121.30915328;
	Gpslong13 = 121.30914202;

	double Gpsla1 = 31.19691151;
	double Gpsla2 = 31.19688899;
	double Gpsla3 = 31.19686509;
	double Gpsla4 = 31.19705386;
	double Gpsla5 = 31.19703155;
	double Gpsla6 = 31.19700724;
	double Gpsla7 = 31.19720759;
	double Gpsla8 = 31.19718445;
	double Gpsla9 = 31.19716105;
	Gpsla10 = 31.19756502;
	Gpsla11 = 31.19753676;
	Gpsla12 = 31.19749911;
	Gpsla13 = 31.19760351;


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
	mp_Gpslong[10] = Gpslong10;
	mp_Gpslong[11] = Gpslong11;
	mp_Gpslong[12] = Gpslong12;
	mp_Gpslong[13] = Gpslong13;

	mp_Gpslat[1] = Gpsla1;
	mp_Gpslat[2] = Gpsla2;
	mp_Gpslat[3] = Gpsla3;
	mp_Gpslat[4] = Gpsla4;
	mp_Gpslat[5] = Gpsla5;
	mp_Gpslat[6] = Gpsla6;
	mp_Gpslat[7] = Gpsla7;
	mp_Gpslat[8] = Gpsla8;
	mp_Gpslat[9] = Gpsla9;
	mp_Gpslat[10] = Gpsla10;
	mp_Gpslat[11] = Gpsla11;
	mp_Gpslat[12] = Gpsla12;
	mp_Gpslat[13] = Gpsla13;

	
	
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

	Point3d p31(-2.800, 48.8, 1.2);
	Point3d p32(-5.800, 48.8, 1.2);
	Point3d p33(-8.600, 48.8, 1.2);
	Point3d p34(-3.000, 82.4, 1.2);
	Point3d p35(-5.800, 82.4, 1.2);
	Point3d p36(-9.000, 82.4, 1.2);
	Point3d p37(-3.000, 117.6, 1.2);
	Point3d p38(-6.200, 117.6, 1.2);
	Point3d p39(-9.000, 117.6, 1.2);
	Point3d p310(-3.000, 200.0, 1.2);
	Point3d p311(-5.800, 200.0, 1.2);
	Point3d p312(-9.000, 200.0, 1.2);
	Point3d p313(2.000, 200.0, 1.2);
	map_Measures[1] = p31;
	map_Measures[2] = p32;
	map_Measures[3] = p33;
	map_Measures[4] = p34;
	map_Measures[5] = p35;
	map_Measures[6] = p36;
	map_Measures[7] = p37;
	map_Measures[8] = p38;
	map_Measures[9] = p39;
	map_Measures[10] = p310;
	map_Measures[11] = p311;
	map_Measures[12] = p312;
	map_Measures[13] = p313;

	m_Calibrations.PickMeasureMentValue4RadarRT(pickPointindex, map_Measures);

	m_WholeCalibrations.PickMeasureMentValue4RadarRT(wholeindex, map_Measures);

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
	char* dt = ctime(&now);
	cout << "本地日期和时间：" << dt << endl;
	outfile << "Current Generate time: \n"  << dt << endl;
	for (int i = 0; i < pickPointindex.size(); ++i)
	{
		outfile << "Pick the Points:" << pickPointindex[i] << " for calibration" << endl;
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

	// 设置相机的内参和畸变系数
	m_Calibrations.SetCameraInstrinic(fx,fy,cx,cy);
	m_Calibrations.SetCameraDiff(-0.37926434, -1.35701064, 0.00229614642, 0.000660113081, 10.8547903);


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
		cv::solvePnPRansac(worldBoxPoints, m_Calibrations.imagePixel_pick, cameraMatrix1, distCoeffs1, rvec1, tvec1, false, SOLVEPNP_ITERATIVE);
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

	bool useRTK = true;
	m_Calibrations.CalibrateCamera(rasac,useRTK, pickPointindex);
	
	
	//cv::solvePnP(worldBoxPoints, boxPoints, cameraMatrix1, distCoeffs1, rvec1, tvec1, false,CV_P3P);
	





	// 反向计算到图像上的点是否正确

		// gps 转化成世界坐标之后的点

		Mat m_gps2world = Mat::ones(4, 1, cv::DataType<double>::type);
	
		//3D to 2D////////////////////////////
		Mat image_points = Mat::ones(3, 1, cv::DataType<double>::type);
		Mat RT_;
		hconcat(m_Calibrations.m_cameraRMatrix33, m_Calibrations.m_cameraTMatrix, RT_);
		cout << "Image RT_" << RT_ << endl;
		outfile << "it is an Image RT matrix then! \n\n" << endl;
		outfile << RT_ << endl;
	

		// 反推九个点投影到pixel坐标
		vector<Point3d>::iterator iter = m_WholeCalibrations.m_worldBoxPoints.begin();
		for (; iter!= m_WholeCalibrations.m_worldBoxPoints.end(); iter++)
		{
			m_gps2world.at<double>(0, 0) = iter->x;
			m_gps2world.at<double>(1, 0) = iter->y;
			// 预估gps测量时的z轴高度为1.2f
			m_gps2world.at<double>(2, 0) = 1.2f;
			image_points = cameraMatrix1 * RT_ * m_gps2world;
			Mat D_Points = Mat::ones(3, 1, cv::DataType<double>::type);
			D_Points.at<double>(0, 0) = image_points.at<double>(0, 0) / image_points.at<double>(2, 0);
			D_Points.at<double>(1, 0) = image_points.at<double>(1, 0) / image_points.at<double>(2, 0);

			cout << "3D to 2D:   " << D_Points << endl;

			Point2d raderpixelPoints;
			raderpixelPoints.x = D_Points.at<double>(0, 0);
			raderpixelPoints.y = D_Points.at<double>(1, 0);
			validPoints.push_back(raderpixelPoints);
			std::string raders = "radarPoints";

			circle(sourceImage, raderpixelPoints, 6, Scalar(0, 0, 255), -1, LINE_AA);
		}

		// 计算标定误差
		for (int i=0; i < validPoints.size(); ++i)
		{
			double error_pixel = std::abs(validPoints[i].x - boxPoints[i].x) / 2560;
			double error_pixel_y = std::abs(validPoints[i].y - boxPoints[i].y) / 1440;
			std::cout <<"Point:"<<i<<"\t"<< "error:X\t"<< error_pixel<<"\t"<< "error:Y\t" <<error_pixel_y<< std::endl;
			outfile << "Point:" << i << "\t" << "error:X\t" << error_pixel << "\t" << "error:Y\t" << error_pixel_y << std::endl;
		}


		// GPS test points to draw in an image
		CalibrationTool &testPoint = CalibrationTool::getInstance();
		vector<double> gpslos{ 121.30811344 };
		vector<double> gpslas{ 31.19713822 };
		testPoint.Gps2WorldCoord(gpslos, gpslas);
		testPoint.SetWorldBoxPoints();
		vector<Point3d>::iterator itertest = testPoint.m_worldBoxPoints.begin();
		for (; itertest != testPoint.m_worldBoxPoints.end(); itertest++)
		{
			m_gps2world.at<double>(0, 0) = itertest->x;
			m_gps2world.at<double>(1, 0) = itertest->y;
			m_gps2world.at<double>(2, 0) = 1.2;
			image_points = m_Calibrations.m_cameraintrisic * RT_ * m_gps2world;
			Mat D_Points = Mat::ones(3, 1, cv::DataType<double>::type);
			D_Points.at<double>(0, 0) = image_points.at<double>(0, 0) / image_points.at<double>(2, 0);
			D_Points.at<double>(1, 0) = image_points.at<double>(1, 0) / image_points.at<double>(2, 0);
			Point2d raderpixelPoints;
			raderpixelPoints.x = D_Points.at<double>(0, 0);
			raderpixelPoints.y = D_Points.at<double>(1, 0);
			
			sprintf(textbuf, "GpsTest");
			//putText(sourceImage, textbuf, Point((int)raderpixelPoints.x - 50, (int)raderpixelPoints.y-10), 0,1.3, Scalar(0,0,255),5);
			//circle(sourceImage, raderpixelPoints, 8, Scalar(100, 20, 0), -1, LINE_AA);
		}

		
		// 手动配置的雷达点

		 // 雷达的原始点 unit 米
		cv::Mat RadarPoint = Mat::ones(4, 1, cv::DataType<double>::type);
		Mat world_point = Mat::ones(4, 1, cv::DataType<double>::type);
		Point3d  radartest,radartest1,radartest2,radartest3;

		/*radartest.x = -2.5;
		radartest.y = 86;
		radartest.z = 1.2;
		radartest1.x = -9;
		radartest1.y = 65.5;
		radartest1.z = 1.2;
		radartest2.x = -8.8;
		radartest2.y = 165.6;
		radartest2.z = 1.2;*/

		radartest3.x = -9.4;
		radartest3.y = 200.8;
		radartest3.z = 1.2;

	
		vector<Point3d> m_radartests;
		/*m_radartests.push_back(radartest);
		m_radartests.push_back(radartest1);
		m_radartests.push_back(radartest2);*/
		m_radartests.push_back(radartest3);
		
		for (int i = 0; i< m_radartests.size(); ++i)
		{
			Point2d raderpixelPoints;
			UcitCalibrate::WorldDistance worldDistance;
			worldDistance.X = m_radartests[i].x;
			worldDistance.Y = m_radartests[i].y;
			worldDistance.Height = m_radartests[i].z;
			m_Calibrations.Distance312Pixel(worldDistance, raderpixelPoints);
			std::string raders = "radarPoints";
			cout << "Pixel_2D_X:\t" << raderpixelPoints.x <<"\t"<<"Pixel_2D_Y:\t"<<raderpixelPoints.y<< endl;
			circle(sourceImage, raderpixelPoints, 10, Scalar(200, 0, 255), -1, LINE_AA);
		}


		outfile.close();

		// 这个是计算pixel 到雷达坐标系
		Point2d  m_Distancepixel(1319, 608);
		circle(sourceImage, m_Distancepixel, 10, Scalar(200, 0, 255), -1, LINE_AA);	
		UcitCalibrate::WorldDistance m_Distance;
		m_Calibrations.Pixel2Distance31(m_Distancepixel, m_Distance);
		sprintf(textbuf, "CeJU:(%.3f,%.3f)",m_Distance.X, m_Distance.Y);
		putText(sourceImage, textbuf, Point((int)m_Distancepixel.x - 80, (int)m_Distancepixel.y - 10), 0, 1.2, Scalar(0, 0, 255), 5);


		// imwrite("F:\\test\\calibration\\roadsideCalibration\\save.jpg", sourceImage);
	imshow("raw image", sourceImage);
	waitKey(0);
    system("PAUSE");
    return 0;
}

