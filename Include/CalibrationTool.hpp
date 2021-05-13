#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <map>
#include "tinystr.h"
#include "tinyxml.h"

using namespace cv;
using namespace std;

namespace UcitCalibrate
{
	struct GpsWorldCoord
	{
		double X;
		double Y;
		double Distance;
	};

	struct WorldDistance
	{
		double X;
		double Y;
		double Height;
	};


	class CalibrationTool
	{

	public:
		static CalibrationTool &getInstance()
		{
			static CalibrationTool wl_UcitCalibration;
			return wl_UcitCalibration;
		};


		struct BoxSize
		{
			int xMin;
			int yMin;
			int xMax;
			int yMax;
		};
		
		bool ReadParaXml(std::string m_strXmlPath, std::vector<BoxSize>& vecNode);

		bool ReadPickpointXml(std::string m_xmlpath, 
			std::vector<unsigned int>& pickpoints, 
			vector<Point2d> &rawpoint, 
			std::map<int, Point2d> &map_points, 
			std::map<int, double> &map_long, 
			std::map<int, double> &map_lan,
			double &reflectheight,
			std::map<int, Point3d> &map_Measures);

		void Gps2WorldCoord(vector<double> P1_lo, vector<double> P1_la);
		void CameraPixel2World(Point2d m_pixels, Point3d &m_world, cv::Mat rotate33);

		// pixel 折算到3*1的距离值, 1: camerapixel points 2：输出x,y,z值，计算像素到雷达坐标系
		void Pixel2Distance31(Point2d pixels, WorldDistance &Distances);
		void Distance312Pixel(WorldDistance Distances, Point2d& pixels);
		void SetWorldBoxPoints();
		vector<Point3d> GetWorldBoxPoints(); 
		// choose selected points for calibration
		void PickRawGPSPoints4VectorPairs(vector<unsigned int> pointsSet, std::map<int, double>&Gps_longtitude, std::map<int, double>&Gps_latitudes);
		void PickMeasureMentValue4RadarRT(vector<unsigned int> pointsSet, std::map<int, Point3d> &measurements);
		void PickImagePixelPoints4PnPsolve(vector<unsigned int>pointsSet, std::map<int, Point2d>& imagesPixel);
		
		vector<Point3d> GetMeasureMentPoint();
		void CalibrateCamera(bool rasac,bool useRTK, vector<unsigned int> pickPoints);
		void SetPi(double pai);
		void SetRadarHeight(double radar_height);
		void SetCameraInstrinic(double fx, double fy, double cx, double cy);
		void SetCameraDiff(double df1, double df2, double df3, double df4, double df5);
		void SetCoordinateOriginPoint(double longitude, double latitude);
		void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
		void codeRotateByY(double x, double z, double thetay, double& outx, double& outz);
		void codeRotateByX(double y, double z, double thetax, double& outy, double& outz);
		cv::Point3d RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta);
		vector<GpsWorldCoord> GetGpsworlds();
		// radar2 world
		cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints);
		cv::Mat GetRadarRTMatrix();
		cv::Mat GetCameraRT44Matrix();
		bool SetCameraRT44(cv::Mat CmRT44);
		bool SetRadarRT44(cv::Mat RadRT44);

		vector<Point3d> m_worldBoxPoints;
		vector<double> gps_longPick;
		vector<double> gps_latiPick;
		vector<Point3d> measures_pick;
		vector<Point2f> imagePixel_pick;
		cv::Mat  m_cameraRMatrix;
		cv::Mat m_cameraRMatrix33;
		cv::Mat  m_cameraTMatrix;
		cv::Mat  m_cameraintrisic;
	
	private:
		CalibrationTool();
		virtual ~CalibrationTool();
		vector<GpsWorldCoord> m_gpsworlds;
		// 用gps计算得到的数据构造以下
		
		double m_PI;
		double m_earthR;
		double m_earthR_Polar;
		double m_originlongitude;
		double m_originlatitude;
		double m_radarheight;	
		cv::Mat  m_cameradiff;
		cv::Mat  m_RadarRT;
		cv::Mat  m_cameraRTMatrix44;
		
	};

};

