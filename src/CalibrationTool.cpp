#include "CalibrationTool.hpp"
namespace UcitCalibrate
{

	CalibrationTool::CalibrationTool()
		: m_PI(3.1415926535898)
		, m_earthR(6378245)
		, m_earthR_Polar(6378245)
		, m_originlongitude(121.30727344f)
		, m_originlatitude(31.19672181f)
		, m_gpsworlds()
		, m_worldBoxPoints()
		, gps_longPick()
		, gps_latiPick()
		, measures_pick()
		, imagePixel_pick()
	{

	};

	CalibrationTool:: ~CalibrationTool()
	{

	};

	void CalibrationTool::SetPi(double pai)
	{
		m_PI = pai;
	}

	void CalibrationTool::SetCoordinateOriginPoint(double longitude, double latitude)
	{
		m_originlongitude = longitude;
		m_originlatitude = latitude;
	}


	void CalibrationTool::Gps2WorldCoord(vector<double> P1_lo, vector<double> P1_la)
	{
		if (P1_la.size()!= P1_lo.size())
		{
			printf("input the longitude and latitude can't be paired!!! return");
			return;
		}
		m_gpsworlds.clear();
	

		double val = m_PI / 180.0;
		//double P1_lo[9] = { 121.30774132, 121.30775749,121.30777151,  121.30804771,121.30806281,121.30807791,  121.30838804,121.30839387,121.30841134 };  //p1,p2,p3,p4
		//double P1_la[9] = { 31.19691151,  31.19688899, 31.19686509,   31.19705386, 31.19703155,31.19700724,   31.19728386,31.19718445, 31.19716105 };

		for (int i = 0; i < P1_la.size(); i++)
		{
			GpsWorldCoord GpsWorldtmp;
			double x;
			double y;
			double dis;
			GpsWorldtmp.X = 2 * m_PI * (m_earthR_Polar * cos(P1_la[i] * val)) * ((P1_lo[i] - m_originlongitude) / 360);
			GpsWorldtmp.Y = 2 * m_PI * m_earthR * ((P1_la[i] - m_originlatitude) / 360);
			GpsWorldtmp.Distance = sqrt(GpsWorldtmp.X * GpsWorldtmp.X + GpsWorldtmp.Y * GpsWorldtmp.Y);
			printf("P[%d]::Gps world:x=%.10f Gps World y=%.10f Distance of world=%.10f \n", i + 1, GpsWorldtmp.X, GpsWorldtmp.Y, \
			GpsWorldtmp.Distance);
			m_gpsworlds.push_back(GpsWorldtmp);
		}
		printf("******************* \n");
		printf("******************* \n");
		printf("******************* \n");
		printf("******************* \n");
	}

	vector<GpsWorldCoord> CalibrationTool::GetGpsworlds()
	{
		return m_gpsworlds;
	}

	void CalibrationTool::SetWorldBoxPoints()
	{
		if (!m_worldBoxPoints.empty())
		{
			m_worldBoxPoints.clear();
		}
		Point3d tmpPoint;
		for (int i = 0; i < m_gpsworlds.size(); i++)
		{
			tmpPoint.x = m_gpsworlds[i].X;
			tmpPoint.y = m_gpsworlds[i].Y;
			tmpPoint.z = 0.0f;
			m_worldBoxPoints.push_back(tmpPoint);
		}
	
	}

	void CalibrationTool::PickRawGPSPoints4VectorPairs(vector<unsigned int> pointsSet, std::map<int, double>& Gps_longtitude, std::map<int, double>& Gps_latitudes)
	{
		if (pointsSet.empty())
		{
			return;
		}

		gps_latiPick.clear();
		gps_longPick.clear();

		std::map<int, double>::iterator itergpslong = Gps_longtitude.begin();
		std::map<int, double>::iterator itergpslat = Gps_latitudes.begin();
		for (int i =0; i< pointsSet.size(); i++)
		{
			itergpslong = Gps_longtitude.find(pointsSet[i]);
			itergpslat = Gps_latitudes.find(pointsSet[i]);
			if (itergpslong != Gps_longtitude.end())
			{
				printf("Pick the Point[%d] for calibration, be attention!! \n", pointsSet[i]);
				gps_longPick.push_back(itergpslong->second);
				gps_latiPick.push_back(itergpslat->second);
			}
		}

	}

	void CalibrationTool::PickMeasureMentValue4RadarRT(vector<unsigned int> pointsSet, std::map<int, Point3d>& measurements)
	{
		if (pointsSet.empty())
		{
			return;
		}
		measures_pick.clear();
		std::map<int, Point3d>::iterator iter	= measurements.begin();
		for (int i = 0; i < pointsSet.size(); i++)
		{
			iter = measurements.find(pointsSet[i]);
			if (iter != measurements.end())
			{
				printf("Pick the Point[%d] for calibration, be attention!! \n", pointsSet[i]);
				measures_pick.push_back(iter->second);
			}
		}
	}

	void CalibrationTool::PickImagePixelPoints4PnPsolve(vector<unsigned int>pointsSet, std::map<int, Point2f>& imagesPixel)
	{
		if (pointsSet.empty())
		{
			return;
		}
		imagePixel_pick.clear();
		std::map<int, Point2f>::iterator iter = imagesPixel.begin();
		for (int i = 0; i < pointsSet.size(); i++)
		{
			iter = imagesPixel.find(pointsSet[i]);
			if (iter != imagesPixel.end())
			{
				printf("Pick the Point[%d] for calibration, be attention!! \n", pointsSet[i]);
				imagePixel_pick.push_back(iter->second);
			}
		}
	}

	vector<Point3d> CalibrationTool::GetWorldBoxPoints()
	{
		return m_worldBoxPoints;
	}

	void CalibrationTool::codeRotateByX(double y, double z, double thetax, double& outy, double& outz)
	{
		double y1 = y;
		double z1 = z;
		double rx = thetax * m_PI/ 180;
		outy = cos(rx) * y1 - sin(rx) * z1;
		outz = cos(rx) * z1 + sin(rx) * y1;
	}

	void CalibrationTool::codeRotateByY(double x, double z, double thetay, double& outx, double& outz)
	{
		double x1 = x;
		double z1 = z;
		double ry = thetay * m_PI / 180;
		outx = cos(ry) * x1 + sin(ry) * z1;
		outz = cos(ry) * z1 - sin(ry) * x1;
	}

	void CalibrationTool::codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
	{
		double x1 = x;
		double y1 = y;
		double rz = thetaz * m_PI / 180;
		outx = cos(rz) * x1 - sin(rz) * y1;
		outy = sin(rz) * x1 + cos(rz) * y1;
	}

	cv::Point3d CalibrationTool::RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
	{
		double r = theta * m_PI / 180;
		double c = cos(r);
		double s = sin(r);
		double new_x = (vx * vx * (1 - c) + c) * old_x + (vx * vy * (1 - c) - vz * s) * old_y + (vx * vz * (1 - c) + vy * s) * old_z;
		double new_y = (vy * vx * (1 - c) + vz * s) * old_x + (vy * vy * (1 - c) + c) * old_y + (vy * vz * (1 - c) - vx * s) * old_z;
		double new_z = (vx * vz * (1 - c) - vy * s) * old_x + (vy * vz * (1 - c) + vx * s) * old_y + (vz * vz * (1 - c) + c) * old_z;
		return cv::Point3d(new_x, new_y, new_z);
	}


	// ����״�����ϵ����������ϵ��R_T����
	cv::Mat CalibrationTool::Get3DR_TransMatrix(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints)
	{
		double srcSumX = 0.0f;
		double srcSumY = 0.0f;
		double srcSumZ = 0.0f;

		double dstSumX = 0.0f;
		double dstSumY = 0.0f;
		double dstSumZ = 0.0f;

		//���������
		if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
		{
			printf("input the radar points are less than three pairs, can't be calculated!!!!");
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
		for (int i = 0; i < pointsNum; ++i)//N���
		{
			//����
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
		double det = cv::determinant(matTemp);//����ʽ��ֵ

		double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
		cv::Mat matM(3, 3, CV_64FC1, datM);

		cv::Mat matR = matV.t() * matM * matU.t();

		double* datR = (double*)(matR.data);
		double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
		double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
		double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);

		//����RT��ξ���(4*4)
		cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
			matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
			matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
			matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
			0, 0, 0, 1
			);

		return R_T;

	}

	
	vector<Point3d> CalibrationTool::GetMeasureMentPoint()
	{
		return measures_pick;
	}

}

