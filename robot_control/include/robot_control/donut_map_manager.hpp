#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP
#include "irl_grid_map.hpp"
#include "global_vars.hpp"
#include <vector>

struct MAP_DATA_T
{
    double singleSampleProb;
    double anySampleProb;
	int roiNum;
};

struct CELL_DATA_T
{
    size_t xIndex;
    size_t yIndex;
	float xPos; // global coordinates, m
	float yPos; // global coordinates, m
	float distanceToRobot; // m
};

class MapManager
{
public:
    // Members
    IRLGridMap<MAP_DATA_T> map;
    global_vars::GLOBAL_VARS_T& global;
    std::vector<std::vector<CELL_DATA_T>>& roiCells;
    // Methods
    MapManager(global_vars::GLOBAL_VARS_T& globalIn, std::vector<std::vector<CELL_DATA_T>>& roiCellsIn, float mapRes, float mapXDim, float mapYDim, int totalNumSamplesIn, int numROIsIn); // Constructor, initializes map with initial probability distribution
    void selectDonutCells(std::vector<CV_OBSERVATION_DATA_T> positiveSamplePositions); // Populates the cellsToNegSmash_ and cellsToPosSmash_ vectors based on the current robot position and any visible positive samples
    void setSampleFoundCells(float sampleXPos, float sampleYPos, int numSamplesFound); // Sets cellsToPosSmash_ to empty and cellsToNegSmash_ to the cells specified
    void donutSmash(int numSamplesFound); // Processes cell indices stored in cellsToNegSmash_ and cellsToPosSmash_ and performs either the positive or negative observation update rule
    // Access single sample probability of any cell
    double getSSProb(size_t xIndex, size_t yIndex);
    double getSSProb(int xIndex, int yIndex);
    double getSSProb(float xPos, float yPos);
    double getSSProb(CELL_DATA_T cell);
    // Set the single sample probability of any cell and update the any sample probability correspondingly
    void setSSProb(size_t xIndex, size_t yIndex, double value, int numFound);
    void setSSProb(int xIndex, int yIndex, double value, int numFound);
    void setSSProb(float xPos, float yPos, double value, int numFound);
    void setSSProb(CELL_DATA_T cell, double value, int numFound);
    // Get any sample probability of any cell
    double getASProb(size_t xIndex, size_t yIndex);
    double getASProb(int xIndex, int yIndex);
    double getASProb(float xPos, float yPos);
    double getASProb(CELL_DATA_T cell);
    // Set the any sample probability of any cell and update the single sample probability correspondingly
    void setASProb(size_t xIndex, size_t yIndex, double value, int numFound);
    void setASProb(int xIndex, int yIndex, double value, int numFound);
    void setASProb(float xPos, float yPos, double value, int numFound);
    void setASProb(CELL_DATA_T cell, double value, int numFound);
    MAP_DATA_T& mapAtCell(CELL_DATA_T cell); // Convenience function to make the code less cumbersome to read when accessing the map via XY indices
private:
    // Members
	std::vector<CELL_DATA_T> cellsToNegSmash_;
	std::vector<CELL_DATA_T> cellsToPosSmash_;
    const int totalNumSamples;
    const int numROIs;
};

#endif // MAP_MANAGER_HPP
