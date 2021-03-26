#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <map>

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


	class CalibrationTool
	{

	public:
		CalibrationTool();
		virtual ~CalibrationTool();
		void Gps2WorldCoord(vector<double> P1_lo, vector<double> P1_la);
		void SetWorldBoxPoints();
		vector<Point3d> GetWorldBoxPoints(); 
		// choose selected points for calibration
		void PickRawGPSPoints4VectorPairs(vector<unsigned int> pointsSet, std::map<int, double>&Gps_longtitude, std::map<int, double>&Gps_latitudes);
		void PickMeasureMentValue4RadarRT(vector<unsigned int> pointsSet, std::map<int, Point3d> &measurements);
		void PickImagePixelPoints4PnPsolve(vector<unsigned int>pointsSet, std::map<int, Point2f>& imagesPixel);
		
		vector<Point3d> GetMeasureMentPoint();
		void SetPi(double pai);
		void SetCoordinateOriginPoint(double longitude, double latitude);
		void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
		void codeRotateByY(double x, double z, double thetay, double& outx, double& outz);
		void codeRotateByX(double y, double z, double thetax, double& outy, double& outz);
		cv::Point3d RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta);
		vector<GpsWorldCoord> GetGpsworlds();
		// radar2 world

		cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints);
		vector<Point3d> m_worldBoxPoints;
		vector<double> gps_longPick;
		vector<double> gps_latiPick;
		vector<Point3d> measures_pick;
		vector<Point2f> imagePixel_pick;
	private:
		vector<GpsWorldCoord> m_gpsworlds;
		// 用gps计算得到的数据构造以下
		
		double m_PI;
		double m_earthR;
		double m_earthR_Polar;
		double m_originlongitude;
		double m_originlatitude;
		
	};

};

