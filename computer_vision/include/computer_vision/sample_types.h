#ifndef SAMPLE_TYPE_H
#define SAMPLE_TYPE_H

#include <string>
#include <vector>

enum SAMPLE_TYPE {_unknown, _white, _whiteBlueOrPurple, _whitePink, _whiteRed, _whiteOrange, _whiteYellow, _silver, _blueOrPurple, _pink, _red, _orange, _yellow, _N_SAMPLE_TYPE};

std::string map_enum_to_string(SAMPLE_TYPE type)
{
	switch(type)
	{
		case _unknown:
			return "unknown";
			break;
		case _white:
			return "white";
			break;
		case _whiteBlueOrPurple:
			return "whiteBlueOrPurple";
			break;
		case _whitePink:
			return "whitePink";
			break;
		case _whiteRed:
			return "whiteRed";
			break;
		case _whiteOrange:
			return "whiteOrange";
			break;
		case _whiteYellow:
			return "whiteYellow";
			break;
		case _silver:
			return "silver";
			break;
		case _blueOrPurple:
			return "blueOrPurple";
			break;
		case _pink:
			return "pink";
			break;
		case _red:
			return "red,";
			break;
		case _orange:
			return "orange";
			break;
		case _yellow:
			return "yellow";
			break;
		default:
			return "unknown";
			break;
	}
}

std::vector<int> map_enum_to_color(SAMPLE_TYPE type)
{
	std::vector<int> scalar_values;
	scalar_values.clear();

	switch(type)
	{
		case _unknown:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
		case _white:
			scalar_values.push_back(255);
			scalar_values.push_back(255);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteBlueOrPurple:
			scalar_values.push_back(255);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
		case _whitePink:
			scalar_values.push_back(147);
			scalar_values.push_back(20);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteRed:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteOrange:
			scalar_values.push_back(0);
			scalar_values.push_back(69);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteYellow:
			scalar_values.push_back(0);
			scalar_values.push_back(215);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _silver:
			scalar_values.push_back(255);
			scalar_values.push_back(200);
			scalar_values.push_back(200);
			return scalar_values;
			break;
		case _blueOrPurple:
			scalar_values.push_back(255);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
		case _pink:
			scalar_values.push_back(147);
			scalar_values.push_back(20);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _red:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _orange:
			scalar_values.push_back(0);
			scalar_values.push_back(69);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _yellow:
			scalar_values.push_back(0);
			scalar_values.push_back(215);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		default:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
	}	
}

#endif // SAMPLE_TYPE_ENUM_H