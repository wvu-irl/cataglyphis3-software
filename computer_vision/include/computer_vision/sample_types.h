#ifndef SAMPLE_TYPE_H
#define SAMPLE_TYPE_H

#include <string>
#include <vector>

//NOTE THE COLOR THRESHOLDS ARE GENERATED IN THE LOOKUP TABLE
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

std::vector<SAMPLE_TYPE_T> map_to_simple_color(SAMPLE_TYPE_T type)
{
	std::vector<SAMPLE_TYPE_T> possible_types;
	possible_types.clear();
	switch(type)
	{
		case _unknown_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_blueOrPurple_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
		case _white_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_blueOrPurple_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
		case _whiteBlueOrPurple_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_blueOrPurple_t);
			break;
		case _whitePink_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			break;
		case _whiteRed_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			break;
		case _whiteOrange_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_orange_t);
			break;
		case _whiteYellow_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
		case _silver_t:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_blueOrPurple_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
		case _blueOrPurple_t:
			// possible_types.push_back(_white_t);
			// possible_types.push_back(_silver_t);
			possible_types.push_back(_blueOrPurple_t);
			break;
		case _pink_t:
			// possible_types.push_back(_white_t);
			// possible_types.push_back(_silver_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			break;
		case _red_t:
			// possible_types.push_back(_white_t);
			// possible_types.push_back(_silver_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			break;
		case _orange_t:
			// possible_types.push_back(_white_t);
			// possible_types.push_back(_silver_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
		case _yellow_t:
			// possible_types.push_back(_white_t);
			// possible_types.push_back(_silver_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
		default:
			possible_types.push_back(_white_t);
			possible_types.push_back(_silver_t);
			possible_types.push_back(_blueOrPurple_t);
			possible_types.push_back(_pink_t);
			possible_types.push_back(_red_t);
			possible_types.push_back(_orange_t);
			possible_types.push_back(_yellow_t);
			break;
	}

	return possible_types;
}

#endif // SAMPLE_TYPE_ENUM_H