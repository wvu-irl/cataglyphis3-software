#include <robot_control/donut_map_manager.hpp>

MapManager::MapManager(global_vars::GLOBAL_VARS_T& globalIn, std::vector<std::vector<CELL_DATA_T> > &roiCellsIn, float mapRes, float mapXDim, float mapYDim, int totalNumSamplesIn, int numROIsIn)
    : map(mapRes, mapXDim, mapYDim),
      totalNumSamples(totalNumSamplesIn),
      numROIs(numROIsIn),
      global(globalIn),
      roiCells(roiCellsIn)
{

}

void MapManager::selectDonutCells(std::vector<CV_OBSERVATION_DATA_T> positiveSamplePositions)
{
    float posSampleXPos;
    float posSampleYPos;
    size_t posSampleXIndex;
    size_t posSampleYIndex;
    size_t robotCellXIndex;
    size_t robotCellYIndex;
    float cellXPos;
    float cellYPos;
    bool addCellToPosList;
    bool addCellToNegList;
    CELL_DATA_T cellToPushBack;
    // Clear and cellsToPosSmash_ vector for new entries
    cellsToPosSmash_.clear();
    // Populate cellsToPosSmash_ with cell indices of locations with positive observations
    for(int i=0; i<positiveSamplePositions.size(); i++)
    {
        // Compute global coordinates of positive sample observation
        posSampleXPos = global.xPos + positiveSamplePositions.at(i).distance*cos(global.heading + positiveSamplePositions.at(i).bearing);
        posSampleYPos = global.yPos + positiveSamplePositions.at(i).distance*sin(global.heading + positiveSamplePositions.at(i).bearing);
        // Find cell indices of that global coordinate
        map.getIndex(posSampleXIndex, posSampleYIndex, posSampleXPos, posSampleYPos);
        // Check if cell has already been added to positive list. Should not have more than one sample in same cell, so one must be false.
        addCellToPosList = true; // Assume initially that it is not a duplicate cell
        for(int j=0; j<cellsToPosSmash_.size(); j++)
        {
            if(posSampleXIndex == cellsToPosSmash_.at(j).xIndex && posSampleYIndex == cellsToPosSmash_.at(j).yIndex)
            {
                addCellToPosList = false;
            }
        }
        // Write that index into the cellsToPosSmash_ vector
        if(addCellToPosList)
        {
            cellToPushBack.xIndex = posSampleXIndex;
            cellToPushBack.yIndex = posSampleYIndex;
            map.getPos(cellXPos, cellYPos, posSampleXIndex, posSampleYIndex);
            cellToPushBack.distanceToRobot = hypot(cellXPos - global.xPos, cellYPos - global.yPos);
            cellsToPosSmash_.push_back(cellToPushBack);
        }
    }
    // Clear and cellsToNegSmash_ vector for new entries
    cellsToNegSmash_.clear();
    // Loop over all cells in the ROIs in the map
    for(int i=0; i<numROIs; i++)
    {
        for(int j=0; j<roiCells.at(i).size(); j++)
        {
            // Check if the cell's position is inside the robot's search radius but not the cell the robot is in, because it cannot be seen
            cellToPushBack = roiCells.at(i).at(j);
            cellToPushBack.distanceToRobot = hypot(cellToPushBack.xPos - global.xPos, cellToPushBack.yPos - global.yPos);
            if(cellToPushBack.distanceToRobot <= global.searchRadius && (cellToPushBack.xIndex!=robotCellXIndex || cellToPushBack.yIndex!=robotCellYIndex))
            {
                // If it is, then check if the cell is one of the cells to be positively smashed
                addCellToNegList = true;
                for(int k=0; k<cellsToPosSmash_.size(); k++)
                {
                    if(i==cellsToPosSmash_.at(k).xIndex && j==cellsToPosSmash_.at(k).yIndex) addCellToNegList = false;
                }
                // If it is not one to be positively smashed, then it can be added to the negative list. Otherwise, it is already in the positive list so it is skipped
                if(addCellToNegList)
                {
                    cellsToNegSmash_.push_back(cellToPushBack);
                }
            }
        }
    }
}

void MapManager::setSampleFoundCells(float sampleXPos, float sampleYPos, int numSamplesFound)
{
    double trueNegative;
    double falseNegative;
    double pNeg;
    double computedProb;
    float cellPosX;
    float cellPosY;
    CELL_DATA_T cellToPushBack;
    cellsToPosSmash_.clear();
    cellsToNegSmash_.clear();
    cellToPushBack.xPos = sampleXPos;
    cellToPushBack.yPos = sampleYPos;
    map.getIndex(cellToPushBack.xIndex, cellToPushBack.yIndex, cellToPushBack.xPos, cellToPushBack.yPos);
    cellToPushBack.distanceToRobot = hypot(cellToPushBack.xPos - global.xPos, cellToPushBack.yPos - global.yPos);
    cellsToNegSmash_.push_back(cellToPushBack);
    for(int k=0; k<cellsToNegSmash_.size(); k++)
    {
        // Get position of the cell
        map.getPos(cellPosX, cellPosY, cellsToNegSmash_.at(k).xIndex, cellsToNegSmash_.at(k).yIndex);
        // Compute probabilities for update functions
        trueNegative = 1.0;
        falseNegative = 0.0;
        pNeg = falseNegative*getSSProb(cellsToNegSmash_.at(k)) + trueNegative*(1.0 - getSSProb(cellsToNegSmash_.at(k)));
        // Update cell being observed
        setSSProb(cellsToNegSmash_.at(k), 0.0, numSamplesFound);
        // Propogate update to all other cells in the map
        for(int i=0; i<numROIs; i++)
        {
            for(int j=0; j<roiCells.at(i).size(); j++)
            {
                // But not the cell that was just smashed
                if(roiCells.at(i).at(j).xIndex!=cellsToNegSmash_.at(k).xIndex || roiCells.at(i).at(j).yIndex!=cellsToNegSmash_.at(k).yIndex)
                {
                    computedProb = trueNegative*getSSProb(roiCells.at(i).at(j))/pNeg;
                    setSSProb(roiCells.at(i).at(j), computedProb, numSamplesFound);
                }
            }
        }
    }
}

void MapManager::donutSmash(int numSamplesFound)
{
    double truePositive;
    double falsePositive;
    double trueNegative;
    double falseNegative;
    double pPos;
    double pNeg;
    double computedProb;
    float cellPosX;
    float cellPosY;
    // Positive smash
    for(int k=0; k<cellsToPosSmash_.size(); k++)
    {
        // Get position of the cell
        map.getPos(cellPosX, cellPosY, cellsToPosSmash_.at(k).xIndex, cellsToPosSmash_.at(k).yIndex);
        // Compute probabilities for update functions
        truePositive = global.tpMax*(1.0 - pow(hypot(cellPosX - global.xPos, cellPosY - global.yPos)/(global.searchRadius + global.searchRadiusProbComputationBuffer), 2.0));
        falsePositive = global.falsePositiveRate;
        trueNegative = 1.0 - falsePositive;
        falseNegative = 1.0 - truePositive;
        pPos = truePositive*getSSProb(cellsToPosSmash_.at(k)) + falsePositive*(1.0 - getSSProb(cellsToPosSmash_.at(k)));
        // Update cell being observed
        //irlPrint("positive smash: cell observed before = AS:%f, SS:%f, ",getASProb(cellsToPosSmash_.at(k)),getSSProb(cellsToPosSmash_.at(k)));
        computedProb = truePositive*getSSProb(cellsToPosSmash_.at(k))/pPos;
        setSSProb(cellsToPosSmash_.at(k), computedProb, numSamplesFound);
        //irlPrint("after = AS:%f, SS:%f\n",getASProb(cellsToPosSmash_.at(k)),getSSProb(cellsToPosSmash_.at(k)));
        //irlPrint("tp = %f\nfp = %f\ntn = %f\nfn = %f\npPos = %f\n",truePositive,falsePositive,trueNegative,falseNegative,pPos);
        //irlPrint("cell at [%f,%f], robot at [%f,%f], distance = %f\n",cellPosX,cellPosY,global.xPos,global.yPos,hypot(cellPosX - global.xPos, cellPosY - global.yPos));
        // Propogate update to all other cells in the map
        for(int i=0; i<numROIs; i++)
        {
            for(int j=0; j<roiCells.at(i).size(); j++)
            {
                // But not the cell that was just smashed
                if(roiCells.at(i).at(j).xIndex!=cellsToPosSmash_.at(k).xIndex || roiCells.at(i).at(j).yIndex!=cellsToPosSmash_.at(k).yIndex)
                {
                    computedProb = falsePositive*getSSProb(roiCells.at(i).at(j))/pPos;
                    setSSProb(roiCells.at(i).at(j), computedProb, numSamplesFound);
                }
            }
        }
    }
    // Negative smash
    for(int k=0; k<cellsToNegSmash_.size(); k++)
    {
        // Get position of the cell
        map.getPos(cellPosX, cellPosY, cellsToNegSmash_.at(k).xIndex, cellsToNegSmash_.at(k).yIndex);
        // Compute probabilities for update functions
        truePositive = global.tpMax*(1.0 - pow(hypot(cellPosX - global.xPos, cellPosY - global.yPos)/(global.searchRadius + global.searchRadiusProbComputationBuffer), 2.0));
        falsePositive = global.falsePositiveRate;
        trueNegative = 1.0 - falsePositive;
        falseNegative = 1.0 - truePositive;
        pNeg = falseNegative*getSSProb(cellsToNegSmash_.at(k)) + trueNegative*(1.0 - getSSProb(cellsToNegSmash_.at(k)));
        // Update cell being observed
        computedProb = falseNegative*getSSProb(cellsToNegSmash_.at(k))/pNeg;
        setSSProb(cellsToNegSmash_.at(k), computedProb, numSamplesFound);
        // Propogate update to all other cells in the map
        for(int i=0; i<numROIs; i++)
        {
            for(int j=0; j<roiCells.at(i).size(); j++)
            {
                // But not the cell that was just smashed
                if(roiCells.at(i).at(j).xIndex!=cellsToNegSmash_.at(k).xIndex || roiCells.at(i).at(j).yIndex!=cellsToNegSmash_.at(k).yIndex)
                {
                    computedProb = trueNegative*getSSProb(roiCells.at(i).at(j))/pNeg;
                    setSSProb(roiCells.at(i).at(j), computedProb, numSamplesFound);
                }
            }
        }
    }
}

double MapManager::getSSProb(size_t xIndex, size_t yIndex)
{
    return map.atIndex(xIndex, yIndex).singleSampleProb;
}

double MapManager::getSSProb(int xIndex, int yIndex)
{
    return getSSProb((size_t)xIndex, (size_t)yIndex);
}

double MapManager::getSSProb(float xPos, float yPos)
{
    return map.atPos(xPos, yPos).singleSampleProb;
}

double MapManager::getSSProb(CELL_DATA_T cell)
{
    return map.atIndex(cell.xIndex, cell.yIndex).singleSampleProb;
}

void MapManager::setSSProb(size_t xIndex, size_t yIndex, double value, int numFound)
{
    map.atIndex(xIndex, yIndex).singleSampleProb = value;
    map.atIndex(xIndex, yIndex).anySampleProb = 1.0 - pow(1.0 - value, totalNumSamples - numFound);
}

void MapManager::setSSProb(int xIndex, int yIndex, double value, int numFound)
{
    setSSProb((size_t)xIndex, (size_t)yIndex, value, numFound);
}

void MapManager::setSSProb(float xPos, float yPos, double value, int numFound)
{
    map.atPos(xPos, yPos).singleSampleProb = value;
    map.atPos(xPos, yPos).anySampleProb = 1.0 - pow(1.0 - value, totalNumSamples - numFound);
}

void MapManager::setSSProb(CELL_DATA_T cell, double value, int numFound)
{
    map.atIndex(cell.xIndex, cell.yIndex).singleSampleProb = value;
    map.atIndex(cell.xIndex, cell.yIndex).anySampleProb = 1.0 - pow(1.0 - value, totalNumSamples - numFound);
}

double MapManager::getASProb(size_t xIndex, size_t yIndex)
{
    return map.atIndex(xIndex, yIndex).anySampleProb;
}

double MapManager::getASProb(int xIndex, int yIndex)
{
    return getASProb((size_t)xIndex, (size_t)yIndex);
}

double MapManager::getASProb(float xPos, float yPos)
{
    return map.atPos(xPos, yPos).anySampleProb;
}

double MapManager::getASProb(CELL_DATA_T cell)
{
    return map.atIndex(cell.xIndex, cell.yIndex).anySampleProb;
}

void MapManager::setASProb(size_t xIndex, size_t yIndex, double value, int numFound)
{
    map.atIndex(xIndex, yIndex).anySampleProb = value;
    map.atIndex(xIndex, yIndex).singleSampleProb = 1.0 - pow(1.0 - value, 1.0/(totalNumSamples - numFound));
}

void MapManager::setASProb(int xIndex, int yIndex, double value, int numFound)
{
    setASProb((size_t)xIndex, (size_t)yIndex, value, numFound);
}

void MapManager::setASProb(float xPos, float yPos, double value, int numFound)
{
    map.atPos(xPos, yPos).anySampleProb = value;
    map.atPos(xPos, yPos).singleSampleProb = 1.0 - pow(1.0 - value, 1.0/(totalNumSamples - numFound));
}

void MapManager::setASProb(CELL_DATA_T cell, double value, int numFound)
{
    map.atIndex(cell.xIndex, cell.yIndex).anySampleProb = value;
    map.atIndex(cell.xIndex, cell.yIndex).singleSampleProb = 1.0 - pow(1.0 - value, 1.0/(totalNumSamples - numFound));
}

MAP_DATA_T& MapManager::mapAtCell(CELL_DATA_T cell)
{
    return map.atIndex(cell.xIndex, cell.yIndex);
}
