#include "GpsKmlGenerator.hpp"

GpsKmlGenerator::GpsKmlGenerator()
{

}

GpsKmlGenerator::~GpsKmlGenerator()
{

}

int GpsKmlGenerator::writegpskml(inputgps* pinputs, int num_groups, int num_points_pergroup, std::string outpath)
{
	if (pinputs == nullptr)
	{
		printf(">>>>>>>>please create gpsinputs successfully!<<<<<<<");
		return -1;
	}
	TiXmlDocument* writeDoc = new TiXmlDocument; //xml文档指针

	//文档格式声明
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "yes");
	writeDoc->LinkEndChild(decl); //写入文档

	TiXmlElement* RootElement = new TiXmlElement("kml");//根元素
	writeDoc->LinkEndChild(RootElement);

	TiXmlElement* Docu = new TiXmlElement("Document");
	RootElement->LinkEndChild(Docu);
	

	//配置三种style 显示
	TiXmlElement* style = new TiXmlElement("Style");
	style->SetAttribute("id", "xuewei_green_Style");
	Docu->LinkEndChild(style);


	TiXmlElement* iconstyle = new TiXmlElement("IconStyle");
	style->LinkEndChild(iconstyle);
	TiXmlElement* scale = new TiXmlElement("scale");
	iconstyle->LinkEndChild(scale);
	TiXmlText* values = new TiXmlText("1");
	scale->LinkEndChild(values);
	TiXmlElement* icon = new TiXmlElement("Icon");
	iconstyle->LinkEndChild(icon);

	TiXmlElement* href = new TiXmlElement("href");
	TiXmlText* green = new TiXmlText("http://maps.google.com/mapfiles/kml/pushpin/grn-pushpin.png");
	href->LinkEndChild(green);
	icon->LinkEndChild(href);

	TiXmlElement* style1 = new TiXmlElement("Style");
	style1->SetAttribute("id", "xuewei_blue_Style");
	Docu->LinkEndChild(style1);
	TiXmlElement* iconstyle1 = new TiXmlElement("IconStyle");
	style1->LinkEndChild(iconstyle1);
	TiXmlElement* scale1 = new TiXmlElement("scale");
	iconstyle1->LinkEndChild(scale1);
	TiXmlText* values1 = new TiXmlText("1");
	scale1->LinkEndChild(values1);
	TiXmlElement* icon1 = new TiXmlElement("Icon");
	iconstyle1->LinkEndChild(icon1);

	TiXmlElement* href1 = new TiXmlElement("href");
	TiXmlText* blue = new TiXmlText("http://maps.google.com/mapfiles/kml/pushpin/blue-pushpin.png");
	href1->LinkEndChild(blue);
	icon1->LinkEndChild(href1);

	//配置红色的style

	TiXmlElement* style2 = new TiXmlElement("Style");
	style2->SetAttribute("id", "xuewei_Red_Style");
	Docu->LinkEndChild(style2);
	TiXmlElement* iconstyle2 = new TiXmlElement("IconStyle");
	style2->LinkEndChild(iconstyle2);
	TiXmlElement* scale2 = new TiXmlElement("scale");
	iconstyle2->LinkEndChild(scale2);
	TiXmlText* values2 = new TiXmlText("1");
	scale2->LinkEndChild(values2);
	TiXmlElement* icon2 = new TiXmlElement("Icon");
	iconstyle2->LinkEndChild(icon2);

	TiXmlElement* href2 = new TiXmlElement("href");
	TiXmlText* red = new TiXmlText("http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png");
	href2->LinkEndChild(red);
	icon2->LinkEndChild(href2);

	//连上各种点 placement
	// 读取 pininputs
	for (int i=0; i<num_groups;i++)
	{
		for (int j=0; j< num_points_pergroup; j++)
		{
			char pointname[256];
			std::string choosestyles="";
			TiXmlElement* r_place = new TiXmlElement("Placemark");
			Docu->LinkEndChild(r_place);
			TiXmlElement* r_name = new TiXmlElement("name");
			r_place->LinkEndChild(r_name);
			switch (pinputs[i].m_type)
			{
			case 0:
				sprintf(pointname, "rawgps:%d", j + 1);
				choosestyles = "#xuewei_green_Style";
				break;
			case 1:
				sprintf(pointname, "pixel2gps:%d", j + 1);
				choosestyles = "xuewei_blue_Style";
				break;
			case 2:
				sprintf(pointname, "radar2gps:%d", j + 1);
				choosestyles = "xuewei_Red_Style";
				break;
			}
			TiXmlText* pointnames = new TiXmlText(pointname);
			r_name->LinkEndChild(pointnames);
			
			//创建stlye url
			TiXmlElement* styleurl = new TiXmlElement("styleUrl");
			r_place->LinkEndChild(styleurl);
			TiXmlText* choosestyle = new TiXmlText(choosestyles.c_str());
			styleurl->LinkEndChild(choosestyle);

			TiXmlElement* Pointdian = new TiXmlElement("Point");
			r_place->LinkEndChild(Pointdian);
			TiXmlElement* coordinates = new TiXmlElement("coordinates");
			Pointdian->LinkEndChild(coordinates);

			// 最终坐标, 经纬度之间拼接个逗号，不要sprintf试试看
			std::ostringstream stream;
			std::string jingdu = dou22str(pinputs[i].v_lonandlat[j].longtitude);
			std::string weidu = dou22str(pinputs[i].v_lonandlat[j].latitude);

			stream << jingdu;
			// 补充一个空格
			stream <<",";
			stream << weidu;
			std::string zuobiaopinjie = stream.str();
			TiXmlText* coordvalues = new TiXmlText(zuobiaopinjie.c_str());
			coordinates->LinkEndChild(coordvalues);
		}
	}



	

	writeDoc->SaveFile(outpath.c_str());
	delete writeDoc;


	return 0;
}


std::string  GpsKmlGenerator::dou22str(double num, int precision)   //num也可以是int类型
{
	stringstream ss;      //stringstream需要sstream头文件
	ss.precision(precision);
	string str;
	ss << num;
	ss >> str;
	return str;
}