

#include "CalibrationTool.hpp"
#include <ctime>
using namespace UcitCalibrate;


void Gps2WorldCoord4test(double earthR,double handleheight, \
	longandlat m_longandlat, std::map<int, double> P1_lo, \
	std::map<int, double> P1_la, vector<Point3d> &m_worldBoxPoints)
{

	if (P1_la.size() != P1_lo.size())
	{
		printf("input the longitude and latitude can't be paired!!! return");
		return;
	}
	double val = CV_PI / 180.0;
	for (int i = 1; i < P1_la.size(); i++)
	{
		Point3d temp3d;
		temp3d.x = 2 * CV_PI * (earthR * cos(P1_la[i] * val)) * ((P1_lo[i] - m_longandlat.longtitude) / 360);
		temp3d.y = 2 * CV_PI * earthR * ((P1_la[i] - m_longandlat.latitude) / 360);
		temp3d.z = handleheight;
		m_worldBoxPoints.push_back(temp3d);
	}
	printf("******************* \n");
	
}




int main()
{
	////// 首先通过标定板的图像像素坐标以及对应的世界坐标，通过PnP求解相机的R&T//////
	// 准备的是图像上的像素点
	// 创建输出参数文件，ios：：trunc含义是有文件先删除
	ofstream outfile("CalibrateLog.txt", ios::trunc);
	std::string m_xmlpath = "input.xml";

	// 基于当前系统的当前日期/时间
	time_t now = time(0);
	// 把 now 转换为字符串形式
	
	
	CalibrationTool  &m_Calibrations = CalibrationTool::getInstance();
	//CalibrationTool  &m_WholeCalibrations = CalibrationTool::getInstance();
	// 设置挑选点
	//vector<unsigned int> pickPointindex{1,2,3,4,5,6,7,8,9,10,11,12};
	vector<unsigned int> pickPointindex;
	bool rasac = false;


	// dont change wholeindex
	vector<unsigned int> wholeindex{ 1,2,3,4,5,6,7,8,9,10,11,12 };
	double reflectorheight,raderheight;
	std::map<int, Point3d> m_Measures;
	vector<Point2d> boxPoints, validPoints;
	std::map<int, Point2d> mp_images;
	std::map<int, double> mp_Gpslong, mp_Gpslat;

	longandlat originallpoll;
	// read from xml config
	m_Calibrations.ReadPickpointXml(m_xmlpath,
		pickPointindex,
		boxPoints,
		mp_images,
		mp_Gpslong,
		mp_Gpslat,
		reflectorheight,
		raderheight,
		m_Measures,
		originallpoll);


	//处理measures
	std::map<int, Point3d>::iterator iter_begin = m_Measures.begin();
	std::map<int, Point3d>::iterator iter_end = m_Measures.end();
	for (; iter_begin != iter_end; iter_begin++)
	{
		double ydist = iter_begin->second.y;
		double ynew = sqrt(ydist * ydist - pow(raderheight - reflectorheight, 2));
		iter_begin->second.y = ynew;
		cout << ynew << "newy of iter:" << iter_begin->second.y << endl;

	}

	//setpi
	m_Calibrations.SetPi(CV_PI);
	m_Calibrations.SetCoordinateOriginPoint(originallpoll.longtitude, originallpoll.latitude);
	m_Calibrations.SetRadarHeight(reflectorheight);
	std::vector<CalibrationTool::BoxSize> mv_results;
	m_Calibrations.PickImagePixelPoints4PnPsolve(pickPointindex, mp_images);
	//m_WholeCalibrations.PickImagePixelPoints4PnPsolve(wholeindex, mp_images);

	// Step one Loading image
	Mat sourceImage = imread("1.bmp");
    // 写字
    
    char textbuf[256];

	// draw 原始的取点
	for (int i = 0; i < boxPoints.size(); ++i)
	{
		circle(sourceImage, boxPoints[i], 8, Scalar(100, 100, 0), -1, LINE_AA);
        int text_x = (int)(boxPoints[i].x - 30);
        int text_y = (int)(boxPoints[i].y - 10);
        sprintf(textbuf, "RPoint%d",i);
        //putText(sourceImage, textbuf, cv::Point(text_x, text_y), FONT_HERSHEY_COMPLEX,3, Scalar(0,0,255));
	}

	namedWindow("raw image", WINDOW_NORMAL);
	
    // Step two Calculate the GPS 2 World
	
	
	// pick points for calibration
	m_Calibrations.PickRawGPSPoints4VectorPairs(pickPointindex, mp_Gpslong, mp_Gpslat);
	//m_WholeCalibrations.PickRawGPSPoints4VectorPairs(wholeindex, mp_Gpslong, mp_Gpslat);

	// gps convert to the world coordination
	m_Calibrations.Gps2WorldCoord(m_Calibrations.gps_longPick, m_Calibrations.gps_latiPick);


	//m_WholeCalibrations.Gps2WorldCoord(m_WholeCalibrations.gps_longPick, m_WholeCalibrations.gps_latiPick);
	//m_WholeCalibrations.SetWorldBoxPoints();

	
	m_Calibrations.SetWorldBoxPoints();


	//Setting box corners in real world//////
	vector<Point3d> worldBoxPoints;  //存入世界坐标
	vector<Point3d> measurementPoint;
	worldBoxPoints = m_Calibrations.GetWorldBoxPoints();

	// 计算雷达的rt矩阵
	// 用于反算雷达是否标定准确,以下的点是实测距离值，单位为米
	vector<Point3d> srcPoints;


	m_Calibrations.PickMeasureMentValue4RadarRT(pickPointindex, m_Measures);

	//m_WholeCalibrations.PickMeasureMentValue4RadarRT(wholeindex, m_Measures);

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
	outfile << "\n" << endl;
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

		outfile << "Rotate33:" << m_Calibrations.m_cameraRMatrix33<<"\n\n"<< endl;
		outfile << "CameraTmatrix:" << m_Calibrations.m_cameraTMatrix << "\n" << endl;
	

		// 反推12个点投影到pixel坐标,以下用新构造的点来计算
		vector<Point3d> m_fantuiceshi;
		Gps2WorldCoord4test(6378245,reflectorheight, originallpoll,mp_Gpslong,mp_Gpslat,m_fantuiceshi);

		vector<Point3d>::iterator iter = m_fantuiceshi.begin();
		for (; iter!= m_fantuiceshi.end(); iter++)
		{
			m_gps2world.at<double>(0, 0) = iter->x;
			m_gps2world.at<double>(1, 0) = iter->y;
			// 预估gps测量时的z轴高度为1.2f
			m_gps2world.at<double>(2, 0) = iter->z;
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

			circle(sourceImage, raderpixelPoints, 6, Scalar(100, 0, 255), -1, LINE_AA);
		}

		// 计算标定误差
		outfile << "\n" << endl;
		outfile << "Error X not exceeding 20, while Error Y not exceeding 15 at least!!\n" << endl;
		vector<int> index;
		for (int i=0; i < validPoints.size(); ++i)
		{
		
			double error_pixel = std::abs(validPoints[i].x - boxPoints[i].x) / 2560;
			double error_x = std::abs(validPoints[i].x - boxPoints[i].x);
			double error_y = std::abs(validPoints[i].y - boxPoints[i].y);
			double error_pixel_y = std::abs(validPoints[i].y - boxPoints[i].y) / 1440;
			if (error_x>20 || error_y>14)
			{
				index.push_back(i);
			}
			std::cout <<"Point:"<<i<<"\t"<< "error:X\t"<< error_x<<"\t"<< "error:Y\t" <<error_y<< std::endl;
			outfile << "Point:" << i << "\t" << "error:X\t" << error_x << "\t" << "error:Y\t" << error_y << std::endl;
		}
		outfile << "\n" << endl;
		for (int k=0;k<index.size();++k)
		{
			outfile << "Point:" << index[k]<<"\t's\t" << "error exceeds requirement!!" << endl;
		}
		outfile << "\n" << endl;
		if (index.size()==3)
		{
			outfile << "there are  three points exceeding the requirement,calibration has just passed!!\n  " << endl;
			outfile << "passed!" << endl;
		}
		else if (index.size()>3)
		{
			outfile << "calibration's accuracy didn't satisfy requirement!!!\n" << endl;
			outfile << "calibrate failed!<<" << endl;
		}
		else if(index.size() < 3)
		{
			outfile << "calibration's accuracy  satisfy requirement,good\n" << endl;
			outfile << "calibrate good enough!" << endl;
		}

		// GPS test points to draw in an image
		/*CalibrationTool &testPoint = CalibrationTool::getInstance();
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
			putText(sourceImage, textbuf, Point((int)raderpixelPoints.x - 50, (int)raderpixelPoints.y-10), 0,1.3, Scalar(0,0,255),5);
			circle(sourceImage, raderpixelPoints, 15, Scalar(255, 20, 20), -1, LINE_AA);
		}*/

		
		// 手动配置的雷达点

		 // 雷达的原始点 unit 米
		cv::Mat RadarPoint = Mat::ones(4, 1, cv::DataType<double>::type);
		Mat world_point = Mat::ones(4, 1, cv::DataType<double>::type);
		Point3d  radartest,radartest1,radartest2,radartest3;

		


		radartest1.x = -4.0;
		radartest1.y = 30.0;
		//radartest1.y = sqrt(30 * 30 + 4.8 * 4.8);
		radartest1.z = 1.2;

		

	
		vector<Point3d> m_radartests;
	
		m_radartests.push_back(radartest1);
	
		
		for (int i = 0; i< m_radartests.size(); ++i)
		{
			Point2d raderpixelPoints;
			UcitCalibrate::WorldDistance worldDistance;
			worldDistance.X = m_radartests[i].x;
			worldDistance.Y = m_radartests[i].y;
			worldDistance.Height = m_radartests[i].z;
			m_Calibrations.Distance312Pixel(worldDistance, raderpixelPoints);
			std::string raders = "radarPoints";
			cout << "给定雷达画图像:\t" << raderpixelPoints.x <<"\t"<<"Pixel_2D_Y:\t"<<raderpixelPoints.y<< endl;
			circle(sourceImage, raderpixelPoints, 10, Scalar(200, 100, 255), -1, LINE_AA);
		}


		outfile.close();

		// 这个是计算pixel 到雷达坐标系
		Point2d  m_Distancepixel(1126, 606);
		cv::Rect2d m_rect(1075,523,97,78);
		cv::rectangle(sourceImage, m_rect, cv::Scalar(255, 255, 0), 4, 8, 0);
		circle(sourceImage, m_Distancepixel, 10, Scalar(200, 0, 255), -1, LINE_AA);	
		UcitCalibrate::WorldDistance m_Distance;
		m_Calibrations.Pixel2Distance31(m_Distancepixel, m_Distance);
		sprintf(textbuf, "Dist:(%.3f,%.3f)",m_Distance.X, m_Distance.Y);
		putText(sourceImage, textbuf, Point((int)m_Distancepixel.x - 80, (int)m_Distancepixel.y - 10), 0, 1, Scalar(0, 0, 255), 3);


	imwrite("save.jpg", sourceImage);
	imshow("raw image", sourceImage);
	waitKey(0);
    system("PAUSE");
    return 0;
}

