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

	//配置highlight 状态

	TiXmlElement* highlightState = new TiXmlElement("Style");
	highlightState->SetAttribute("id", "highlightState");
	Docu->LinkEndChild(highlightState);
	TiXmlElement* highlighticon = new TiXmlElement("IconStyle");
	highlightState->LinkEndChild(highlighticon);
	TiXmlElement* highscale = new TiXmlElement("scale");
	highlighticon->LinkEndChild(highscale);
	TiXmlText* valueshigh = new TiXmlText("1.2");
	highscale->LinkEndChild(valueshigh);
	TiXmlElement* iconhigh = new TiXmlElement("Icon");
	highlighticon->LinkEndChild(iconhigh);

	TiXmlElement* hrefhigh = new TiXmlElement("href");
	TiXmlText* pink = new TiXmlText("http://maps.google.com/mapfiles/kml/pushpin/pink-pushpin.png");
	hrefhigh->LinkEndChild(pink);
	iconhigh->LinkEndChild(hrefhigh);

	// 制作styleMap

	TiXmlElement* greenMap = new TiXmlElement("StyleMap");
	greenMap->SetAttribute("id", "greenMapstyle");
	Docu->LinkEndChild(greenMap);
	TiXmlElement* pairs = new TiXmlElement("Pair");
	greenMap->LinkEndChild(pairs);
	TiXmlElement* key = new TiXmlElement("key");
	TiXmlText* keyvalue = new TiXmlText("normal");
	key->LinkEndChild(keyvalue);
	pairs->LinkEndChild(key);
	TiXmlElement* greenMapstyleurl = new TiXmlElement("styleUrl");
	TiXmlText* normalstyle = new TiXmlText("#xuewei_green_Style");
	greenMapstyleurl->LinkEndChild(normalstyle);
	pairs->LinkEndChild(greenMapstyleurl);

	TiXmlElement* highpair = new TiXmlElement("Pair");
	greenMap->LinkEndChild(highpair);
	TiXmlElement* highkey = new TiXmlElement("key");
	TiXmlText* hightkeyvalue = new TiXmlText("highlight");
	highkey->LinkEndChild(hightkeyvalue);
	highpair->LinkEndChild(highkey);
	TiXmlElement* highkeystyleurl = new TiXmlElement("styleUrl");
	TiXmlText* highlightstatusvalue = new TiXmlText("#highlightState");
	highkeystyleurl->LinkEndChild(highlightstatusvalue);
	highpair->LinkEndChild(highkeystyleurl);


	TiXmlElement* blueMap = new TiXmlElement("StyleMap");
	blueMap->SetAttribute("id", "blueMapstyle");
	Docu->LinkEndChild(blueMap);
	TiXmlElement* bluepairs = new TiXmlElement("Pair");
	blueMap->LinkEndChild(bluepairs);
	TiXmlElement* bluekey = new TiXmlElement("key");
	TiXmlText* bluekeyvalue = new TiXmlText("normal");
	bluekey->LinkEndChild(bluekeyvalue);
	bluepairs->LinkEndChild(bluekey);
	TiXmlElement* blueMapstyleurl = new TiXmlElement("styleUrl");
	TiXmlText* bluenormalstyle = new TiXmlText("#xuewei_blue_Style");
	blueMapstyleurl->LinkEndChild(bluenormalstyle);
	bluepairs->LinkEndChild(blueMapstyleurl);

	TiXmlElement* bluehighpair = new TiXmlElement("Pair");
	blueMap->LinkEndChild(bluehighpair);
	TiXmlElement* bluehighkey = new TiXmlElement("key");
	TiXmlText* bluehightkeyvalue = new TiXmlText("highlight");
	bluehighkey->LinkEndChild(bluehightkeyvalue);
	bluehighpair->LinkEndChild(bluehighkey);
	TiXmlElement* bluehighkeystyleurl = new TiXmlElement("styleUrl");
	TiXmlText* bluehighlightstatusvalue = new TiXmlText("#highlightState");
	bluehighkeystyleurl->LinkEndChild(bluehighlightstatusvalue);
	bluehighpair->LinkEndChild(bluehighkeystyleurl);




	TiXmlElement* redMap = new TiXmlElement("StyleMap");
	redMap->SetAttribute("id", "redMapstyle");
	Docu->LinkEndChild(redMap);
	TiXmlElement* redpairs = new TiXmlElement("Pair");
	redMap->LinkEndChild(redpairs);
	TiXmlElement* redkey = new TiXmlElement("key");
	TiXmlText* redkeyvalue = new TiXmlText("normal");
	redkey->LinkEndChild(redkeyvalue);
	redpairs->LinkEndChild(redkey);
	TiXmlElement* redMapstyleurl = new TiXmlElement("styleUrl");
	TiXmlText* rednormalstyle = new TiXmlText("#xuewei_Red_Style");
	redMapstyleurl->LinkEndChild(rednormalstyle);
	redpairs->LinkEndChild(redMapstyleurl);

	TiXmlElement* redhighpair = new TiXmlElement("Pair");
	redMap->LinkEndChild(redhighpair);
	TiXmlElement* redhighkey = new TiXmlElement("key");
	TiXmlText* redhightkeyvalue = new TiXmlText("highlight");
	redhighkey->LinkEndChild(redhightkeyvalue);
	redhighpair->LinkEndChild(redhighkey);
	TiXmlElement* redhighkeystyleurl = new TiXmlElement("styleUrl");
	TiXmlText* redhighlightstatusvalue = new TiXmlText("#highlightState");
	redhighkeystyleurl->LinkEndChild(redhighlightstatusvalue);
	redhighpair->LinkEndChild(redhighkeystyleurl);

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
				sprintf(pointname, "RG:%d", j + 1);
				choosestyles = "#greenMapstyle";
				break;
			case 1:
				sprintf(pointname, "PG:%d", j + 1);
				choosestyles = "#blueMapstyle";
				break;
			case 2:
				sprintf(pointname, "RaG:%d", j + 1);
				choosestyles = "#redMapstyle";
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