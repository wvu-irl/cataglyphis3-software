#include ""
#include "math.h"

class Verification
{
public:

	struct cell
	{
		float x_mean;
		float y_mean;
		float z_mean;
		float var_z;
	};



	verification();
	int nearest_point(cell cell_reading, std::vector<cell> cells_ref);
	bool height_verify(cell read, cell ref);

protected:


	float cal_dist(cell a, cell b);


}

float dist(cell a, cell b)
{
	return sqrt((a.x_mean - b.x_mean) * (a.x_mean - b.x_mean) + (a.y_mean - b.y_mean) * (a.y_mean - b.y_mean));
}

int nearest_point(cell cell_reading, std::vector<cell> cells_ref)
{
	float min_dist = 0;
	int index_pair;
	for(int i = 0; i < cells_ref.sizes(); i++)
	{
		float distance = dist(cell_reading, cells_ref[i]);
		if (min_dist > distance)
		{
			min_dist = distance;
			index_pair = i;
		}
	}

	return index_pair;
}

bool height_verify(cell read, cell ref)
{
	
}