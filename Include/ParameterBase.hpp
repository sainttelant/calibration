#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include "tinystr.h"
#include "tinyxml.h"

using namespace std;
namespace parameter
{
	struct vehicledimension
	{
		int vehicle_width;
		int vehicle_height;
		int vehicle_long;
	};


	class ParameterBase
	{
	public:
		ParameterBase();
		virtual ~ParameterBase();

	protected:
	private:


	};
	
	
}
