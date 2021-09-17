#include "tinystr.h"
#include "tinyxml.h"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <map>
#include <sstream>
#include "iomanip"

using namespace std;

struct longandlat
{
	double longtitude;
	double latitude;
};

struct inputgps
{
	// 0: rawgps 1: pixel2gps 2: radar2gps
	int m_type;
	char m_color[10];
	longandlat* v_lonandlat;
};


class GpsKmlGenerator
{
	public:
		static GpsKmlGenerator& Instance()
		{
			static GpsKmlGenerator xw_GpsKml;
			return xw_GpsKml;
		};

		int writegpskml(inputgps* pinputs,int num_groups, int num_points_pergroup, std::string outpath);
		std::string dou22str(double num, int precision = 16);
	protected:
	private:
		GpsKmlGenerator();
		virtual	~GpsKmlGenerator();
};