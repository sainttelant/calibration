

#include "CalibrationTool.hpp"
#include "ParameterBase.hpp"
#include <ctime>

//using namespace std;
#include "point.h"
#include "lof.h"
#include <algorithm>
#include <map>


using namespace LOF;
using namespace matplot;


using namespace UcitCalibrate;

//#define readcalib
//#define Readcalibratexml 
#define  calibrateradar
#define writecalibratexml
#define point_10


#define project2pixeltest 1
#define project2gpstest 1



enum pixelDirection
{
	topleft = 1,
	topright,
	bottomleft,
	bottomright,
	Nonbias
};

struct dircs
{
	pixelDirection e_dirs;
	std::string m_drirs;
};

dircs judgedirection(Point2d& srcpoint, Point2d& basepoint, double threashold)
{
	dircs ret{Nonbias,"Nonbiase"};
	double xx = srcpoint.x - basepoint.x;
	double yy = srcpoint.y - basepoint.y;

	double offset = std::max(abs(xx), abs(yy));
	if (offset > threashold)
	{
		if (xx < 0 && yy < 0)
		{
			ret.e_dirs = topleft;
			ret.m_drirs = "topleft";
		}
		if (xx > 0 && yy < 0)
		{
			ret.e_dirs = topright;
			ret.m_drirs = "topright";
		}
		if (xx > 0 && yy > 0)
		{
			ret.e_dirs = bottomright;
			ret.m_drirs = "bottomright";
		}
		if (xx < 0 && yy>0)
		{
			ret.e_dirs = bottomleft;
			ret.m_drirs = "bottomleft";
		}
	}
	else
	{
		return ret;
	}
	
	return ret;
}


typedef std::pair<string, double> PAIR;
struct cmpsort {
	bool operator()(const PAIR& lhs, const PAIR& rhs) {
		return lhs.second > rhs.second;
	}
};


string dou2str(double num,int precision = 16)   //num也可以是int类型
{
	stringstream ss;      //stringstream需要sstream头文件
	ss.precision(precision);
	string str;
	ss << num;
	ss >> str;
	return str;
}

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
	for (int i = 1; i < P1_la.size()+1; i++)
	{
		Point3d temp3d;
		temp3d.x = 2 * CV_PI * (earthR * cos(m_longandlat.latitude * val)) * ((P1_lo[i] - m_longandlat.longtitude) / 360);
		temp3d.y = 2 * CV_PI * earthR * ((P1_la[i] - m_longandlat.latitude) / 360);
		temp3d.z = handleheight;
		m_worldBoxPoints.push_back(temp3d);
	}
	printf("******************* \n");
	
}

// 创建xml文件

int writeXmlFile(cv::Mat *raderRT44,
	cv::Mat *cameraRT44,
	cv::Mat *cameraRT33,
	cv::Mat *cameraRT31,
	double m_longtitude,
	double m_latititude,
	cv::Mat *cameraDist,
	cv::Mat *camerainstrinic,
	double handheight,
	double radarinstallheight)
{
	if (raderRT44==nullptr || cameraRT44==nullptr)
	{
		printf("input rt matrix error \n");
		return -1;
	}
	TiXmlDocument* writeDoc = new TiXmlDocument; //xml文档指针

	//文档格式声明
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "yes");
	writeDoc->LinkEndChild(decl); //写入文档

	TiXmlElement* RootElement = new TiXmlElement("CalibInfo");//根元素
	writeDoc->LinkEndChild(RootElement);

	// 雷达rt矩阵写入
	TiXmlElement* RadElement = new TiXmlElement("radarRT44");
	RootElement->LinkEndChild(RadElement);
	for (int r = 0; r < raderRT44->rows; r++)
	{
		for (int c = 0; c < raderRT44->cols; c++)
		{
			TiXmlElement* index = new TiXmlElement("index");
			RadElement->LinkEndChild(index);
			std::string valuestring = dou2str(raderRT44->at<double>(r, c));
			TiXmlText* value = new TiXmlText(valuestring.c_str());
			index->LinkEndChild(value);
		}
	}
	// 摄像头rt矩阵写入
	TiXmlElement* CameraElement = new TiXmlElement("cameraRT44");
	RootElement->LinkEndChild(CameraElement);
	for (int r = 0; r < cameraRT44->rows; r++)
	{
		for (int c = 0; c < cameraRT44->cols; c++)
		{
			TiXmlElement* index = new TiXmlElement("indexcamera");
			CameraElement->LinkEndChild(index);
			std::string valuestring = dou2str(cameraRT44->at<double>(r, c));
			TiXmlText* value = new TiXmlText(valuestring.c_str());
			index->LinkEndChild(value);
		}
	}

	for (int i = 0; i < 4; i++)
	{
		TiXmlElement* index = new TiXmlElement("indexcamera");
		CameraElement->LinkEndChild(index);
		if (i!=3)
		{
			std::string value = "0";
			TiXmlText* values = new TiXmlText(value.c_str());
			index->LinkEndChild(values);
		}
		else
		{
			std::string value = "1";
			TiXmlText* values = new TiXmlText(value.c_str());
			index->LinkEndChild(values);
		}
	}

	TiXmlElement* CameraElement33 = new TiXmlElement("cameraRT33");
	RootElement->LinkEndChild(CameraElement33);
	for (int r = 0; r < cameraRT33->rows; r++)
	{
		for (int c = 0; c < cameraRT33->cols; c++)
		{
			TiXmlElement* index = new TiXmlElement("indexcamera33");
			CameraElement33->LinkEndChild(index);
			std::string valuestring = dou2str(cameraRT33->at<double>(r, c));
			TiXmlText* value = new TiXmlText(valuestring.c_str());
			index->LinkEndChild(value);
		}
	}

	TiXmlElement* CameraElement31 = new TiXmlElement("cameraRT31");
	RootElement->LinkEndChild(CameraElement31);
	for (int r = 0; r < cameraRT31->rows; r++)
	{
		for (int c = 0; c < cameraRT31->cols; c++)
		{
			TiXmlElement* index = new TiXmlElement("indexcamera31");
			CameraElement31->LinkEndChild(index);
			std::string valuestring = dou2str(cameraRT31->at<double>(r, c));
			TiXmlText* value = new TiXmlText(valuestring.c_str());
			index->LinkEndChild(value);
		}
	}


	TiXmlElement* originpoll = new TiXmlElement("originpoll");
	RootElement->LinkEndChild(originpoll);
	for (int r = 0; r < 2; r++)
	{
		if (r ==0 )
		{
			TiXmlElement* lng = new TiXmlElement("lng");
			originpoll->LinkEndChild(lng);
			std::string value = dou2str(m_longtitude);
			TiXmlText* values = new TiXmlText(value.c_str());
			lng->LinkEndChild(values);
		}
		else
		{
			TiXmlElement* lat = new TiXmlElement("lat");
			originpoll->LinkEndChild(lat);
			std::string value = dou2str(m_latititude);
			TiXmlText* values = new TiXmlText(value.c_str());
			lat->LinkEndChild(values);
		}
	}


	TiXmlElement* distort = new TiXmlElement("distort");
	RootElement->LinkEndChild(distort);
	char zifu[256];
	for (int i=0; i < 5;i++)
	{
		sprintf(zifu, "value%d", i);
		TiXmlElement* index = new TiXmlElement(zifu);
		distort->LinkEndChild(index);
		std::string value = dou2str(cameraDist->at<double>(i, 0));
		TiXmlText* values = new TiXmlText(value.c_str());
		index->LinkEndChild(values);
	}

	TiXmlElement* camerainstri = new TiXmlElement("camerainstrinic");
	RootElement->LinkEndChild(camerainstri);

	for (int i=0;i < 4;i++)
	{
		std::string value = "";
		
			if (i == 0)
			{
				value = dou2str(camerainstrinic->at<double>(0, 0));
				TiXmlElement* fx = new TiXmlElement("fx");
				camerainstri->LinkEndChild(fx);
				TiXmlText* values = new TiXmlText(value.c_str());
				fx->LinkEndChild(values);
			}
			else if (i == 1)
			{
				value = dou2str(camerainstrinic->at<double>(1, 1));
				TiXmlElement* fy = new TiXmlElement("fy");
				camerainstri->LinkEndChild(fy);
				TiXmlText* values = new TiXmlText(value.c_str());
				fy->LinkEndChild(values);
			}
			else if (i ==2)
			{
				value = dou2str(camerainstrinic->at<double>(0, 2));
				TiXmlElement* cx = new TiXmlElement("cx");
				camerainstri->LinkEndChild(cx);
				TiXmlText* values = new TiXmlText(value.c_str());
				cx->LinkEndChild(values);
			}
			else
			{
				value = dou2str(camerainstrinic->at<double>(1, 2));
				TiXmlElement* cy = new TiXmlElement("cy");
				camerainstri->LinkEndChild(cy);
				TiXmlText* values = new TiXmlText(value.c_str());
				cy->LinkEndChild(values);
			}
		}

	TiXmlElement* handradarheight = new TiXmlElement("handradarheight");
	RootElement->LinkEndChild(handradarheight);
	TiXmlElement* height = new TiXmlElement("height");
	handradarheight->LinkEndChild(height);
	std::string handheight_str = dou2str(handheight);
	TiXmlText* values = new TiXmlText(handheight_str.c_str());
	height->LinkEndChild(values);

	TiXmlElement* radarinstall = new TiXmlElement("radarinstallheight");
	RootElement->LinkEndChild(radarinstall);
	TiXmlElement* heightrader = new TiXmlElement("height");
	radarinstall->LinkEndChild(heightrader);
	std::string installheights = dou2str(radarinstallheight);
	TiXmlText* isntallvalues = new TiXmlText(installheights.c_str());
	heightrader->LinkEndChild(isntallvalues);


		writeDoc->SaveFile("calibration2.xml");
		delete writeDoc;

		return 1;
}












int main()
{
	////// 首先通过标定板的图像像素坐标以及对应的世界坐标，通过PnP求解相机的R&T//////
	// 准备的是图像上的像素点
	// 创建输出参数文件，ios：：trunc含义是有文件先删除
	ofstream outfile("CalibrateLog.txt", ios::trunc);
	// record GPS files for display

	ofstream gpsfile("gpsGenerator.txt", ios::trunc);


	std::string m_xmlpath = "input.xml";   //原来的标定
	
	// 基于当前系统的当前日期/时间
	time_t now = time(0);
	// 把 now 转换为字符串形式
	


	
	CalibrationTool  &m_Calibrations = CalibrationTool::getInstance();
	//CalibrationTool  &m_WholeCalibrations = CalibrationTool::getInstance();
	// 设置挑选点
	//vector<unsigned int> pickPointindex{1,2,3,4,5,6,7,8,9,10,11,12};
	vector<unsigned int> pickPointindex;
	bool rasac = true;


	// dont change wholeindex
	vector<unsigned int> wholeindex{ 1,2,3,4,5,6,7,8,9,10,11,12 };
	double reflectorheight,raderheight;
	std::map<int, Point3d> m_Measures;
	vector<Point2d> boxPoints, validPoints;
	



	struct meandistance
	{
		double pixeldistance;
		double radardistance;
		double gpsdistance;
	};

	



	vector<meandistance> error_means;

	std::map<int, Point2d> mp_images;
	std::map<int, double> mp_Gpslong, mp_Gpslat;

	longandlat originallpoll;
	cv::Mat  cameradistort1,camrainst;
	vector<double> m_ghostdis;
	std::vector<double> gpsheight;
	// read from xml config

#ifndef  readcalib

		m_Calibrations.ReadPickpointXml(m_xmlpath,
		pickPointindex,
		boxPoints,
		mp_images,
		mp_Gpslong,
		mp_Gpslat,
		reflectorheight,
		raderheight,
		m_Measures,
		originallpoll, m_ghostdis, camrainst,gpsheight);

		//处理measures
		std::map<int, Point3d>::iterator iter_begin = m_Measures.begin();
		std::map<int, Point3d>::iterator iter_end = m_Measures.end();
		/*for (; iter_begin != iter_end; iter_begin++)
		{
			double ydist = iter_begin->second.y;
			double ynew = sqrt(ydist * ydist - pow(raderheight - reflectorheight, 2));
			iter_begin->second.y = ynew;
			cout << ynew << "newy of iter:" << iter_begin->second.y << endl;

		}*/

		m_Calibrations.SetCoordinateOriginPoint(originallpoll.longtitude, originallpoll.latitude);
		m_Calibrations.SetRadarHeight(reflectorheight);

#endif
	



#ifdef Readcalibratexml
		cv::Mat m_rt44, m_crt44, m_crt33, m_crt31;
		double radargaodu, shouchigao;
		m_Calibrations.ReadCalibrateParam(m_calixml, m_rt44, m_crt44, m_crt33, m_crt31,
			originallpoll, m_ghostdis, camrainst,shouchigao,radargaodu);

		for (int i =0; i< m_ghostdis.size(); i++)
		
		{
			printf("dist %f \n", m_ghostdis[i]);
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				double value = camrainst.at<double>(i, j);
				printf("instrinc values:[%.16f] \n", value);
			}
		}
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				double value = m_rt44.at<double>(i, j);
				printf("rader44 values:[%.16f] \n", value);
			}
		}

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				double value = m_crt44.at<double>(i, j);
				printf("camera34 values:[%.16f] \n", value);
			}
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				double value = m_crt33.at<double>(i, j);
				printf("camera33 values:[%.16f] \n", value);
			}
		}


		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 1; j++)
			{
				double value = m_crt31.at<double>(i, j);
				printf("camera31 values:[%.16f] \n", value);
			}
		}
		// 测试转eigen
		Eigen::Matrix4d raderRT;
		cv::cv2eigen(m_rt44, raderRT);
#endif

	

	

	//setpi
	m_Calibrations.SetPi(CV_PI);

	std::vector<CalibrationTool::BoxSize> mv_results;
	m_Calibrations.PickImagePixelPoints4PnPsolve(pickPointindex, mp_images);
	//m_WholeCalibrations.PickImagePixelPoints4PnPsolve(wholeindex, mp_images);

	// Step one Loading image
	Mat sourceImage = cv::imread("1.bmp");

	Mat xiezitu(cv::Size(800, 1440),CV_8UC3);
    // 写字
    
    char textbuf[256];

	// draw 原始的取点
	vector<Point2d>::iterator iter_origpixel = boxPoints.begin();
	vector<Point2d>::iterator iter_origpixel_e = boxPoints.end();

	for (int i = 0; i < boxPoints.size(); ++i)
	{
		circle(sourceImage, boxPoints[i], 8, Scalar(100, 100, 0), -1, LINE_AA);
        int text_x = (int)(boxPoints[i].x - 30);
        int text_y = (int)(boxPoints[i].y - 10);
        sprintf(textbuf, "RP:%d",i+1);
        putText(sourceImage, textbuf, cv::Point(text_x-5, text_y-8), 0,1, Scalar(0,0,255),2,1);
	}

	namedWindow("raw image", WINDOW_NORMAL);
	
    // Step two Calculate the GPS 2 World
	
	
	// pick points for calibration
	m_Calibrations.PickRawGPSPoints4VectorPairs(pickPointindex, mp_Gpslong, mp_Gpslat);
	//m_WholeCalibrations.PickRawGPSPoints4VectorPairs(wholeindex, mp_Gpslong, mp_Gpslat);

	// gps convert to the world coordination
	// 11号点的高度信息

#ifdef point_11
	double gps0_height = 200.500;
	double point1_height = 200.591 - gps0_height;
	double point2_height = 200.574 - gps0_height;
	double point3_height = 200.476 - gps0_height;
	double point4_height = 200.394 - gps0_height;
	double point5_height = 200.305 - gps0_height;
	double point6_height = 200.478 - gps0_height;
	double point7_height = 200.576 - gps0_height;
	double point8_height = 200.564 - gps0_height;
	double point9_height = 200.591 - gps0_height;
	double point10_height = 200.902 - gps0_height;
	double point11_height = 200.935 - gps0_height;
	double point12_height = 201.253 - gps0_height;
	double point13_height = 201.403 - gps0_height;
	double point14_height = 201.906 - gps0_height;
	double point15_height = 201.750 - gps0_height;
	double point16_height = 202.194 - gps0_height;
	double point17_height = 202.359 - gps0_height;
	double point18_height = 201.191 - gps0_height;
	double point19_height = 201.947 - gps0_height;
	double point20_height = 201.755 - gps0_height;
	double point21_height = 201.089 - gps0_height;

	//  11号点的高度
	//gpsheight.push_back(point1_height);
	//gpsheight.push_back(point2_height);
	//gpsheight.push_back(point3_height);
	//gpsheight.push_back(point4_height);
	//gpsheight.push_back(point5_height);
	//gpsheight.push_back(point6_height);
	//gpsheight.push_back(point7_height);
	//gpsheight.push_back(point8_height);
	//gpsheight.push_back(point9_height);
	//gpsheight.push_back(point10_height);
	gpsheight.push_back(point11_height);
	//gpsheight.push_back(point12_height);
	/*gpsheight.push_back(point13_height);
	gpsheight.push_back(point14_height);
	gpsheight.push_back(point15_height);*/
	//gpsheight.push_back(point16_height);
	gpsheight.push_back(point17_height);
	gpsheight.push_back(point18_height);
	gpsheight.push_back(point19_height);
	gpsheight.push_back(point20_height);
	gpsheight.push_back(point21_height);


#endif


	// 最终选定的高程的点
	std::vector<double> gpsheights;
	for (int j = 0; j < pickPointindex.size(); j++)
	{
		gpsheights.push_back(gpsheight[pickPointindex[j] - 1]);
	}



	m_Calibrations.Gps2WorldCoord(m_Calibrations.gps_longPick, m_Calibrations.gps_latiPick);
	//m_Calibrations.Gps2worldcalib(m_Calibrations.gps_longPick,m_Calibrations.gps_latiPick, gpsheights);
	// 增加手持gps

	

	std::vector<cv::Point3d> Gpsworld4radarcalibrate;
	m_Calibrations.Generategps2world(m_Calibrations.gps_longPick,
		m_Calibrations.gps_latiPick,
		gpsheights,
		Gpsworld4radarcalibrate);


	m_Calibrations.SetWorldBoxPoints();


	//Setting box corners in real world//////
	vector<Point3d> worldBoxPoints;  //存入世界坐标
	vector<Point3d> measurementPoint;
	worldBoxPoints = m_Calibrations.GetWorldBoxPoints();

	// 计算雷达的rt矩阵
	// 用于反算雷达是否标定准确,以下的点是实测距离值，单位为米
	vector<Point3d> srcPoints;




// 标定雷达的rt矩阵
#ifdef calibrateradar
	m_Calibrations.PickMeasureMentValue4RadarRT(pickPointindex, m_Measures);
	// 计算雷达的rt矩阵
	//cv::Mat RT = m_Calibrations.Get3DR_TransMatrix(m_Calibrations.GetMeasureMentPoint(), \
		//m_Calibrations.GetWorldBoxPoints());

	cv::Mat RT = m_Calibrations.Get3DR_TransMatrix(m_Calibrations.GetMeasureMentPoint(), \
		Gpsworld4radarcalibrate);

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
	outfile << "Current Generate time: \n" << dt << endl;
	for (int i = 0; i < pickPointindex.size(); ++i)
	{
		outfile << "Pick the Points:" << pickPointindex[i] << " for calibration" << endl;
	}
	outfile << "\n" << endl;
	outfile << "it is a radar RT matrix first! \n" << endl;
	outfile << RT << "\n\n" << endl;


	printf("**************************************\n");
	printf("**************************************\n");
	printf("**************************************\n");
	printf("**************************************\n");
#endif
	
		
	

	// camera relevant 
	// camera inside parameters
   

	// 设置相机的内参和畸变系数
	m_Calibrations.SetCameraInstrinic(camrainst.at<double>(0,0)
		,camrainst.at<double>(1,1),
		camrainst.at<double>(0,2)
		,camrainst.at<double>(1,2));

	cv::Mat cameradistort = cv::Mat::eye(5, 1, DataType<double>::type);
	for (int i = 0; i < 5; i++)
	{
		cameradistort.at<double>(i, 0) = m_ghostdis[i];
	}

	m_Calibrations.SetCameraDiff(
		cameradistort.at<double>(0,0),
		cameradistort.at<double>(1, 0),
		cameradistort.at<double>(2, 0),
		cameradistort.at<double>(3, 0),
		cameradistort.at<double>(4, 0)
	);
	
	
	
    // 获得世界坐标系到像素坐标系的R_T矩阵
	 //////PnP solve R&T///////////////////////////////
	cv::Mat rvec1(3, 1, cv::DataType<double>::type);  //旋转向量
	cv::Mat tvec1(3, 1, cv::DataType<double>::type);  //平移向量




	m_Calibrations.SetRadarHeight(reflectorheight);
	m_Calibrations.SetCoordinateOriginPoint(originallpoll.longtitude, originallpoll.latitude);

#ifndef calibrateradar
	m_Calibrations.SetRadarRT44(m_rt44);
#endif


	bool useRTK = true;
	m_Calibrations.CalibrateCamera(rasac, useRTK, pickPointindex);




#ifdef readcalib
	// 全是使用读取的参数写进去
	m_Calibrations.SetRadarHeight(1.2);
	m_Calibrations.SetRadarRT44(m_rt44);
	m_Calibrations.SetCameraRT44(m_crt44);
	m_Calibrations.SetCameraRT33(m_crt33);
	m_Calibrations.SetCameraTMatrix(m_crt31);
	m_Calibrations.SetCoordinateOriginPoint(originallpoll.longtitude, originallpoll.latitude);

	

#endif

#if 1
	RadarSpeed m_radar;
	RadarHeading m_radarr;
	m_radar.vx = -3;
	m_radar.vy = 0;
	m_Calibrations.RadarSpeedHeading(m_radar, m_radarr);

#endif
	
	

		// gps 转化成世界坐标之后的点

		Mat m_gps2world = Mat::ones(4, 1, cv::DataType<double>::type);
	
		//3D to 2D////////////////////////////
		Mat image_points = Mat::ones(3, 1, cv::DataType<double>::type);
		Mat RT_;
		hconcat(m_Calibrations.m_cameraRMatrix33, m_Calibrations.m_cameraTMatrix, RT_);
		cout << "Image RT_" << RT_ << endl;
		outfile << "it is an Image RT matrix then! \n\n" << endl;
		outfile << RT_ <<"\n"<< endl;

		outfile << "Rotate33:" << m_Calibrations.m_cameraRMatrix33<<"\n\n"<< endl;
		outfile << "CameraTmatrix:" << m_Calibrations.m_cameraTMatrix << "\n" << endl;
	
		
#ifdef writecalibratexml

		double poll_lon, poll_lat;
		poll_lon = originallpoll.longtitude;
		poll_lat = originallpoll.latitude;
		// generate xml files to store rt matrix
#ifndef Readcalibratexml
		int flag = writeXmlFile(&RT, &RT_, &m_Calibrations.m_cameraRMatrix33, \
			& m_Calibrations.m_cameraTMatrix,
			poll_lon,poll_lat,&cameradistort,&camrainst,reflectorheight,raderheight);
#else
		
		int flag = writeXmlFile(&m_rt44, &RT_, &m_Calibrations.m_cameraRMatrix33, \
			& m_Calibrations.m_cameraTMatrix, poll_lon, poll_lat, &cameradistort, &camrainst, \
			reflectorheight, raderheight);
#endif
		
#endif
		


#if project2pixeltest
		meandistance recorddistance{ 0,0,0 };
		// 反推12个点投影到pixel坐标,以下用新构造的点来计算
		vector<Point3d> m_fantuiceshi;
		Gps2WorldCoord4test(6378245, reflectorheight, originallpoll, mp_Gpslong, mp_Gpslat, m_fantuiceshi);

		vector<Point3d>::iterator iter = m_fantuiceshi.begin();
		for (; iter != m_fantuiceshi.end(); iter++)
		{
			m_gps2world.at<double>(0, 0) = iter->x;
			m_gps2world.at<double>(1, 0) = iter->y;
			// 预估gps测量时的z轴高度为1.2f
			m_gps2world.at<double>(2, 0) = iter->z;
			image_points = camrainst * RT_ * m_gps2world;
			Mat D_Points = Mat::ones(3, 1, cv::DataType<double>::type);
			D_Points.at<double>(0, 0) = image_points.at<double>(0, 0) / image_points.at<double>(2, 0);
			D_Points.at<double>(1, 0) = image_points.at<double>(1, 0) / image_points.at<double>(2, 0);

			cout << "3D to 2D:   " << D_Points << endl;

			Point2d raderpixelPoints;
			raderpixelPoints.x = D_Points.at<double>(0, 0);
			raderpixelPoints.y = D_Points.at<double>(1, 0);
			validPoints.push_back(raderpixelPoints);
			std::string raders = "radarPoints";

			cv::circle(sourceImage, raderpixelPoints, 6, Scalar(0, 0, 255), -1, LINE_AA);
		}
		for (int i = 0; i < boxPoints.size(); i++)
		{
			recorddistance.pixeldistance = sqrt(pow((validPoints[i].x - boxPoints[i].x), 2) + pow((validPoints[i].y - boxPoints[i].y), 2));

			error_means.push_back(recorddistance);
			printf("distancepixle[%d]:%f \n", i, recorddistance.pixeldistance);
		}
#endif
		



		// 计算标定误差
		outfile << "\n" << endl;
		outfile << "Error X not exceeding 20, while Error Y not exceeding 15 at least!!\n" << endl;
		vector<int> index;
		int count = 0;
		for (int i=0; i < validPoints.size(); ++i)
		{
			count++;
			printf("count:%d \n", count);
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

	
#if  project2pixeltest

		// 雷达反投影,应该也要先乘以雷达的逆矩阵
		//  雷达坐标系返回的点是左正右负, 反投影雷达要加高度z，记住
		std::vector<Point2d> radarprojectpixle;
		for (; iter_begin != iter_end; iter_begin++)
		{
			Point3d radartmp;
			radartmp.x = iter_begin->second.x;
			radartmp.y = iter_begin->second.y;
			radartmp.z = iter_begin->second.z;
			Point2d radpixe;

			longandlat gpsresult;
			GpsWorldCoord radar_input, radarworld;
			radar_input.X = radartmp.x;
			radar_input.Y = radartmp.y;
			// input distance 有可能要修改
			radar_input.Distance = 0;
			// 反算雷达到gps
			m_Calibrations.radarworld2Gps(radar_input, gpsresult);
			printf("GPS:[%3.8f,%3.8f] \n", gpsresult.longtitude, gpsresult.latitude);
			printf("GPS:[%3.8f,%3.8f] \n", gpsresult.longtitude, gpsresult.latitude);
			printf("GPS:[%3.8f,%3.8f] \n", gpsresult.longtitude, gpsresult.latitude);

			// 其实是为了得到雷达的Z，这绕了一步
			m_Calibrations.Gps2radarworld(gpsresult, radarworld);

			UcitCalibrate::WorldDistance radardistance;
			radardistance.X = radarworld.X;
			radardistance.Y = radarworld.Y;
			radardistance.Height = radarworld.Distance;

			m_Calibrations.Distance312Pixel(radardistance, radpixe);
			cv::circle(sourceImage, radpixe, 8, Scalar(0, 0, 100), -1, LINE_8);
			radarprojectpixle.push_back(radpixe);

		}

		char dayindir[256];
		char prezifu[50];
		//计算与真值的偏差
		for (int i = 0; i < boxPoints.size(); i++)
		{
			dircs m_dirs = judgedirection(radarprojectpixle[i], boxPoints[i], 20);
			recorddistance.radardistance = sqrt(pow((radarprojectpixle[i].x - boxPoints[i].x), 2) + pow((radarprojectpixle[i].y - boxPoints[i].y), 2));
			error_means[i].radardistance = recorddistance.radardistance;
			printf("distance radarproject[%d]:%f \n", i, recorddistance.radardistance);
			strcpy(dayindir, m_dirs.m_drirs.c_str());
			int textnum = sizeof(m_dirs.m_drirs);
			sprintf(prezifu, "for Point:%d", i + 1);
			//strcpy(dayindir+textnum, prezifu);
			strcat(dayindir, "----->");
			strcat(dayindir, prezifu);
			cv::putText(xiezitu, dayindir, Point(100, 100 + i * 50), 0, 1, Scalar(255, 0, 0), 3);

		}
		cv::namedWindow("debugoffset", 0);
		cv::imshow("debugoffset", xiezitu);
		cv::waitKey(0);
		cv::destroyWindow("debugoffset");

#endif

		


		// 测试航向角,正北为X正，东为世界的Y正 ：假如往西南方向跑
		RadarSpeed m_raderspeed;
		m_raderspeed.vx = 4;
		m_raderspeed.vy = -8;
		m_raderspeed.vz = 0;
		RadarHeading m_heads;
		m_Calibrations.RadarSpeedHeading(m_raderspeed,m_heads);

		

#if project2pixeltest
		std::vector<Point2d> gpsprojects;
		//测试之前采的rtk  gps绝对坐标准不准,GPS反投影,绿色点
		for (int i=1; i < mp_Gpslat.size()+1; i++)
		{
			GpsWorldCoord  m_gps;
			longandlat m_longandtemp;
			m_longandtemp.longtitude = mp_Gpslong[i];
			m_longandtemp.latitude = mp_Gpslat[i];

			m_Calibrations.Gps2radarworld(m_longandtemp, m_gps);
			Point3d radartemp;
			radartemp.x = m_gps.X;
			radartemp.y = m_gps.Y;
			radartemp.z = m_gps.Distance;
			
			Point2d radpixell;
			UcitCalibrate::WorldDistance worldDistance;
			worldDistance.X = radartemp.x;
			worldDistance.Y = radartemp.y;
			worldDistance.Height = radartemp.z;
			m_Calibrations.Distance312Pixel(worldDistance, radpixell);
			std::string raders = "radarPoints";
			sprintf(textbuf, "GPS%d[%3.8f,%3.8f]",i, m_longandtemp.longtitude, m_longandtemp.latitude);
			//putText(sourceImage, textbuf, Point((int)radpixell.x - 100, (int)radpixell.y - 30), 0, 1, Scalar(0, 0, 0), 2);
			cout << "GPS:\t"<< i <<"'s pixels:"<< radpixell.x << "\t" << radpixell.y << endl;
			circle(sourceImage, radpixell, 7, Scalar(0, 255, 0), -1, LINE_AA);
			gpsprojects.push_back(radpixell);
		}

		for (int i = 0; i < boxPoints.size(); i++)
		{
			recorddistance.gpsdistance = sqrt(pow((gpsprojects[i].x - boxPoints[i].x), 2) + pow((gpsprojects[i].y - boxPoints[i].y), 2));
			error_means[i].gpsdistance = recorddistance.gpsdistance;
			printf("distance GPSproject[%d]:%f \n", i, recorddistance.gpsdistance);
		}


		// sum up overall errors 
		std::map<string, double> anverage_distance;
		vector<meandistance>::iterator iter_mean_b = error_means.begin();
		vector<meandistance>::iterator iter_mean_e = error_means.end();
		int counts = 1;
		char strpoint[256];
		for (; iter_mean_b!=iter_mean_e;iter_mean_b++)
		{
			double anveragedistance = (iter_mean_b->pixeldistance + iter_mean_b->radardistance + iter_mean_b->gpsdistance) / 3;
			sprintf(strpoint, "Point:%d", counts);
			anverage_distance.insert(make_pair(strpoint, anveragedistance));
			counts++;
		}


		// 将误差结果排序
		vector<PAIR> name_score_vec(anverage_distance.begin(), anverage_distance.end());
		//对vector排序  
		sort(name_score_vec.begin(), name_score_vec.end(), cmpsort());

		//排序前  
		map<string, double>::iterator iter_map;
		cout << "排序前:" << endl;
		for (iter_map = anverage_distance.begin(); iter_map != anverage_distance.end(); iter_map++)
			cout << iter_map->first <<"\t"<< iter_map->second << endl;


		char resultsdrawing[256];
		cout << "排序后:" << endl;
		for (int i = 0; i != name_score_vec.size(); ++i) {
			//可在此对按value排完序之后进行操作  
			cout << name_score_vec[i].first << "\t"<< name_score_vec[i].second << endl;

			std::ostringstream stream;
			stream << name_score_vec[i].first;
			// 补充一个空格
			stream << setw(10) << setfill(' ');
			stream << name_score_vec[i].second;
			std::string display =stream.str();
			cv::putText(sourceImage, display, Point(50, 200+i*50), 0, 1.5, Scalar(100, 255, 0), 3);
		}	
#endif

#if project2gpstest

		// 先搞三大组的gps
		int pointsize = m_Measures.size();
		inputgps* mp_inputs =(inputgps*) malloc(sizeof(inputgps) * 3);
		for (int i =0; i<3;i++)
		{
			mp_inputs[i].v_lonandlat = (longandlat*)malloc(sizeof(longandlat) * pointsize);
		}
		longandlat tmp;

		mp_Gpslong, mp_Gpslat;
		std::map<int, double>::iterator iter_gpslon = mp_Gpslong.begin();
		std::map<int, double>::iterator iter_gpslat = mp_Gpslat.begin();

	
		gpsfile << "This is RawGPS showing as follows!" << "\n" << endl;
		int counterr = 0;
		for (; iter_gpslon != mp_Gpslong.end() && iter_gpslat != mp_Gpslat.end(); iter_gpslon++, iter_gpslat++)
		{
			printf("RawGPS[%.8f,%.8f] \n", iter_gpslon->second, iter_gpslat->second);
			gpsfile << std::setprecision(12) << iter_gpslat->second << "," << iter_gpslon->second << endl;
			strcpy(mp_inputs[0].m_color, "blue");
			mp_inputs[0].m_type = 0;
			mp_inputs[0].v_lonandlat[counterr].latitude = iter_gpslat->second;
			mp_inputs[0].v_lonandlat[counterr].longtitude = iter_gpslon->second;

			counterr++;
		}


		//  先计算图像到世界
		cv::Point3d m_worldcoord;
		longandlat pixel_gps,radar_gps;
		gpsfile << "This is Pixel2Gps showing as follows!" << "\n" << endl;
		int counterp = 0;
		for (; iter_origpixel != iter_origpixel_e; iter_origpixel++)
		{
			//m_Calibrations.CameraPixel2World(*iter_origpixel,m_worldcoord);
			//printf("m_worldcoord[%.5f,%.5f,%.5f] \n", m_worldcoord.x, m_worldcoord.y,m_worldcoord.z);
			// 再转到GPS 坐标
			m_Calibrations.CameraPixel2Gps(*iter_origpixel, pixel_gps);
			printf("pixel2GPS[%.8f,%.8f] \n", pixel_gps.longtitude, pixel_gps.latitude);
			gpsfile << std::setprecision(12) << pixel_gps.latitude << "," << pixel_gps.longtitude << endl;
			tmp.latitude = pixel_gps.latitude;
			tmp.longtitude = pixel_gps.longtitude;
			strcpy(mp_inputs[1].m_color, "green");
			mp_inputs[1].m_type = 1;
			mp_inputs[1].v_lonandlat[counterp].latitude = pixel_gps.latitude;
			mp_inputs[1].v_lonandlat[counterp].longtitude = pixel_gps.longtitude;
			counterp++;
		}


		gpsfile << "This is radar2GPS showing as follows!" << "\n" << endl;
		// 计算从雷达原始到GPS
		int counteradar = 0;

		std::map<int, Point3d>::iterator iter_begin1 = m_Measures.begin();
		std::map<int, Point3d>::iterator iter_end1 = m_Measures.end();
		for (; iter_begin1 != iter_end1; iter_begin1++)
		{
			Point3d radartmp;
			radartmp.x = iter_begin1->second.x;
			radartmp.y = iter_begin1->second.y;
			radartmp.z = iter_begin1->second.z;
			Point2d radpixe;
			GpsWorldCoord radar_input, radarworld;
			radar_input.X = radartmp.x;
			radar_input.Y = radartmp.y;
			// input distance 有可能要修改
			radar_input.Distance = 0;
			// 反算雷达到gps
			m_Calibrations.radarworld2Gps(radar_input, radar_gps);
			printf("radar2GPS:[%3.8f,%3.8f] \n", radar_gps.longtitude, radar_gps.latitude);

			strcpy(mp_inputs[2].m_color, "red");
			mp_inputs[2].m_type = 2;
			mp_inputs[2].v_lonandlat[counteradar].longtitude = radar_gps.longtitude;
			mp_inputs[2].v_lonandlat[counteradar].latitude = radar_gps.latitude;
			counteradar++;
			gpsfile << std::setprecision(12) << radar_gps.latitude << "," << radar_gps.longtitude << endl;
		}

		// 写入KML files
		std::string m_filepath = "gps.KML";
		GpsKmlGenerator::Instance().writegpskml(mp_inputs,3, pointsize,m_filepath);

		for (int j=0; j<3;j++)
		{
			if (mp_inputs[j].v_lonandlat!=nullptr)
			{
				free(mp_inputs[j].v_lonandlat);
				mp_inputs[j].v_lonandlat == nullptr;
			}
		}
		if (mp_inputs!=nullptr)
		{
			free(mp_inputs);
			mp_inputs == nullptr;
		}
		
		std::cout << "gps thing process done!!" << std::endl;

#endif

#if 1
		Point2d  m_Distancepixel;
		// 计算边界值，y = 0的对应的区域
		Point2d raderpixelPoints, point1, pointend;
		for (float i = -100; i < 100; i += 0.5)
		{
			UcitCalibrate::WorldDistance worldDistance;
			worldDistance.X = i;
			worldDistance.Y = 0;
			worldDistance.Height = 1.2;
			if (i == -9)
			{
				m_Calibrations.Distance312Pixel(worldDistance, point1);
				raderpixelPoints = point1;
			}
			if (i == 8.0)
			{
				m_Calibrations.Distance312Pixel(worldDistance, pointend);
				raderpixelPoints = pointend;
			}
			m_Calibrations.Distance312Pixel(worldDistance, raderpixelPoints);
			std::string raders = "radarPoints";

			cv::circle(sourceImage, raderpixelPoints, 9, Scalar(0, 100, 0), -1, LINE_AA);
			sprintf(textbuf, "%3.1f", i);
			cv::putText(sourceImage, textbuf, Point((int)raderpixelPoints.x - 80, (int)raderpixelPoints.y - 30), 0, 1, Scalar(0, 0, 255), 2);

		}
		BlindArea results;
		cv::line(sourceImage, pointend, point1, Scalar(100, 0, 200), 3);


#endif
		
	

	cv::imwrite("save.jpg", sourceImage);
	cv::imshow("raw image", sourceImage);
	cv::waitKey(0);
	cv::destroyAllWindows();
	outfile.close();
	gpsfile.close();
    std::system("PAUSE");
    return 0;
}

