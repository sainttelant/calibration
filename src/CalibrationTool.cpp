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
		, m_radarheight(0)
		, m_cameraintrisic()
		, m_cameraRMatrix()
		, m_cameraTMatrix()
		, m_cameradiff()
		, m_RadarRT()
		, m_cameraRMatrix33()
		, m_cameraRTMatrix44()
	{

	};

	CalibrationTool:: ~CalibrationTool()
	{

	};

	void CalibrationTool::SetPi(double pai)
	{
		m_PI = pai;
	}

	void CalibrationTool::SetRadarHeight(double radar_height)
	{
		m_radarheight = radar_height;
	}

	void CalibrationTool::SetCoordinateOriginPoint(double longitude, double latitude)
	{
		m_originlongitude = longitude;
		m_originlatitude = latitude;
	}

	void CalibrationTool::SetCameraInstrinic(double fx, double fy, double cx, double cy)
	{
		m_cameraintrisic = Mat::eye(3, 3, cv::DataType<double>::type);
		m_cameraintrisic.at<double>(0, 0) = fx;
		m_cameraintrisic.at<double>(1, 1) = fy;
		m_cameraintrisic.at<double>(0, 2) = cx;
		m_cameraintrisic.at<double>(1, 2) = cy;
	}

	void CalibrationTool::SetCameraDiff(double df1, double df2, double df3, double df4, double df5)
	{
		m_cameradiff = Mat::eye(5, 1, cv::DataType<double>::type);
		m_cameradiff.at<double>(0, 0) = df1;
		m_cameradiff.at<double>(1, 0) = df2;
		m_cameradiff.at<double>(2, 0) = df3;
		m_cameradiff.at<double>(3, 0) = df4;
		m_cameradiff.at<double>(4, 0) = df5;
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
			// Ԥ�������߶�Ϊ1.2f
			tmpPoint.z = 1.2f;
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

	void CalibrationTool::PickImagePixelPoints4PnPsolve(vector<unsigned int>pointsSet, std::map<int, Point2d>& imagesPixel)
	{
		if (pointsSet.empty())
		{
			return;
		}
		imagePixel_pick.clear();
		std::map<int, Point2d>::iterator iter = imagesPixel.begin();
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

		m_RadarRT = R_T;
		return R_T;

	}

	cv::Mat CalibrationTool::GetRadarRTMatrix()
	{
		return m_RadarRT;
	}

	cv::Mat CalibrationTool::GetCameraRT44Matrix()
	{
		return m_cameraRTMatrix44;
	}

	bool CalibrationTool::SetCameraRT44(cv::Mat CmRT44)
	{
		if (CmRT44.cols!=4 && CmRT44.rows!=4)
		{
			printf("Set cameraRT matrix failed!!!!!!!\n");
			return false;
		}
		else
		{
			m_cameraRTMatrix44 = CmRT44;
			printf("Manual Set cameraRT successful!!!!!!!\n");
		}
		return true;

	}

	bool CalibrationTool::SetRadarRT44(cv::Mat RadRT44)
	{
		if (RadRT44.cols != 4 && RadRT44.rows != 4)
		{
			printf("Set RadarRT matrix failed!!!!!!!\n");
			return false;
		}
		else
		{
			m_RadarRT = RadRT44;
			printf("Manual Set RadarRT successful!!!!!!!\n");
		}
		return true;
	}

	vector<Point3d> CalibrationTool::GetMeasureMentPoint()
	{
		return measures_pick;
	}

	void CalibrationTool::CameraPixel2World(Point2d m_pixels, Point3d& m_world, cv::Mat rotate33)
	{
		double s;
		/////////////////////2D to 3D///////////////////////
		cv::Mat imagepixel = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
		imagepixel.at<double>(0, 0) = m_pixels.x;
		imagepixel.at<double>(1, 0) = m_pixels.y;

		Mat tempMat = rotate33.inv() * m_cameraintrisic.inv() * imagepixel;
		Mat tempMat2 = rotate33.inv() * m_cameraTMatrix;
	
		s = m_radarheight + tempMat2.at<double>(2, 0);
		s /= tempMat.at<double>(2, 0);
		cout << "s : " << s << endl;

		Mat camera_cordinates = -rotate33.inv() * m_cameraTMatrix;
		Mat wcPoint = rotate33.inv() * (m_cameraintrisic.inv() * s * imagepixel - m_cameraTMatrix);
		m_world.x = wcPoint.at<double>(0, 0);
		m_world.y = wcPoint.at<double>(1, 0);
		m_world.z = wcPoint.at<double>(2, 0);
		cout << "Pixel2World :" << wcPoint << endl;
	}

	void CalibrationTool::CalibrateCamera(bool rasac, bool useRTK, vector<unsigned int> pickPoints)
	{
		cv::Mat rvec1(3, 1, cv::DataType<double>::type);  //��ת����
		cv::Mat tvec1(3, 1, cv::DataType<double>::type);  //ƽ������
		m_cameraRMatrix = Mat::zeros(3, 1, cv::DataType<double>::type);
		m_cameraTMatrix = Mat::zeros(3, 1, cv::DataType<double>::type);
		m_cameraRMatrix33 = Mat::zeros(3, 3, cv::DataType<double>::type);
		//���� pnp solve ����
		if (useRTK)
		{
			if (rasac)
			{
				cv::solvePnPRansac(m_worldBoxPoints, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
					m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
			}
			else
			{
				if (pickPoints.size() > 4)
				{
					cv::solvePnP(m_worldBoxPoints, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
				}
				else
				{
					cv::solvePnP(m_worldBoxPoints, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_P3P);
				}
			}
		}
		else
		{
			if (rasac)
			{
				cv::solvePnPRansac(measures_pick, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
					m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
			}
			else
			{
				if (pickPoints.size() > 4)
				{
					cv::solvePnP(measures_pick, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
				}
				else
				{
					cv::solvePnP(measures_pick, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_P3P);
				}
			}
		}
		cv::Rodrigues(m_cameraRMatrix, m_cameraRMatrix33);
		hconcat(m_cameraRMatrix33, m_cameraTMatrix, m_cameraRTMatrix44);
	}

	void CalibrationTool::Pixel2Distance31(Point2d pixels, WorldDistance &Distances)
	{
	
		cv::Point3d tmp;
		CameraPixel2World(pixels, tmp, m_cameraRMatrix33);
		cv::Mat Distance_W4 = Mat::ones(4, 1, cv::DataType<double>::type);
		Point3d Distance_world;
		Distance_W4.at<double>(0, 0) = tmp.x;
		Distance_W4.at<double>(1, 0) = tmp.y;
		Distance_W4.at<double>(2, 0) = tmp.z;
		Distance_W4.at<double>(3, 0) = 0;
		cv::Mat radar_Dis = Mat::ones(4, 1, cv::DataType<double>::type);
		radar_Dis = m_RadarRT.inv() * Distance_W4;
		cout << "Distance:X" << radar_Dis.at<double>(0, 0) <<"\t"<< "distance:Y" << radar_Dis.at<double>(1, 0) << endl;
		Distances.X = radar_Dis.at<double>(0, 0);
		Distances.Y = radar_Dis.at<double>(1, 0);
		Distances.Height = radar_Dis.at<double>(2.0);
	}

	void CalibrationTool::Distance312Pixel(WorldDistance Distances, Point2d& pixels)
	{
		cv::Mat RadarPoint = Mat::ones(4, 1, cv::DataType<double>::type);
		Mat world_point = Mat::ones(4, 1, cv::DataType<double>::type);
		Mat imagetmp = Mat::ones(3, 1, cv::DataType<double>::type);
		RadarPoint.at<double>(0, 0) = Distances.X;
		RadarPoint.at<double>(1, 0) = Distances.Y;
		RadarPoint.at<double>(2, 0) = Distances.Height;
		world_point = m_RadarRT * RadarPoint;
		imagetmp = m_cameraintrisic * m_cameraRTMatrix44 * world_point;
		//image_points = m_Calibrations.m_cameraintrisic * RT_ * RadarPoint;
		Mat D_Points = Mat::ones(3, 1, cv::DataType<double>::type);
		D_Points.at<double>(0, 0) = imagetmp.at<double>(0, 0) / imagetmp.at<double>(2, 0);
		D_Points.at<double>(1, 0) = imagetmp.at<double>(1, 0) / imagetmp.at<double>(2, 0);
		
		
		pixels.x = D_Points.at<double>(0, 0);
		pixels.y = D_Points.at<double>(1, 0);
		std::string raders = "radarPoints";
		cout << "Pixel_2D_X:\t" << pixels.x << "\t" << "Pixel_2D_Y:\t" << pixels.y << endl;
		//circle(sourceImage, raderpixelPoints, 10, Scalar(200, 0, 255), -1, LINE_AA);
	}

}

