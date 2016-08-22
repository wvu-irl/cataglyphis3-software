#ifndef SAMPLE_TYPE_H
#define SAMPLE_TYPE_H

#include <string>
#include <vector>

enum SAMPLE_TYPE_T {_unknown_t, _white_t, _whiteBlueOrPurple_t, _whitePink_t, _whiteRed_t, _whiteOrange_t, _whiteYellow_t, _silver_t, _blueOrPurple_t, _pink_t, _red_t, _orange_t, _yellow_t, _N_SAMPLE_TYPE_T};

std::string map_enum_to_string(SAMPLE_TYPE_T type)
{
	switch(type)
	{
		case _unknown_t:
			return "unknown";
			break;
		case _white_t:
			return "white";
			break;
		case _whiteBlueOrPurple_t:
			return "whiteBlueOrPurple";
			break;
		case _whitePink_t:
			return "whitePink";
			break;
		case _whiteRed_t:
			return "whiteRed";
			break;
		case _whiteOrange_t:
			return "whiteOrange";
			break;
		case _whiteYellow_t:
			return "whiteYellow";
			break;
		case _silver_t:
			return "silver";
			break;
		case _blueOrPurple_t:
			return "blueOrPurple";
			break;
		case _pink_t:
			return "pink";
			break;
		case _red_t:
			return "red,";
			break;
		case _orange_t:
			return "orange";
			break;
		case _yellow_t:
			return "yellow";
			break;
		default:
			return "unknown";
			break;
	}
}

std::vector<int> map_enum_to_color(SAMPLE_TYPE_T type)
{
	std::vector<int> scalar_values;
	scalar_values.clear();

	switch(type)
	{
		case _unknown_t:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
		case _white_t:
			scalar_values.push_back(255);
			scalar_values.push_back(255);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteBlueOrPurple_t:
			scalar_values.push_back(255);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
		case _whitePink_t:
			scalar_values.push_back(147);
			scalar_values.push_back(20);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteRed_t:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteOrange_t:
			scalar_values.push_back(0);
			scalar_values.push_back(69);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _whiteYellow_t:
			scalar_values.push_back(0);
			scalar_values.push_back(215);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _silver_t:
			scalar_values.push_back(255);
			scalar_values.push_back(200);
			scalar_values.push_back(200);
			return scalar_values;
			break;
		case _blueOrPurple_t:
			scalar_values.push_back(255);
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			return scalar_values;
			break;
		case _pink_t:
			scalar_values.push_back(147);
			scalar_values.push_back(20);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _red_t:
			scalar_values.push_back(0);
			scalar_values.push_back(0);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _orange_t:
			scalar_values.push_back(0);
			scalar_values.push_back(69);
			scalar_values.push_back(255);
			return scalar_values;
			break;
		case _yellow_t:
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