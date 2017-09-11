#ifndef GLOBAL_VARS_HPP
#define GLOBAL_VARS_HPP
#include <vector>

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

//#define ENABLE_DEBUG_PRINTS

#ifdef ENABLE_DEBUG_PRINTS
#define irlPrint(...) printf(__VA_ARGS__)
#else
#define irlPrint(...)
#endif // ENABLE_DEBUG_PRINTS

struct CV_OBSERVATION_DATA_T
{
    float distance; // robot body coordinates, m
    float bearing; // robot body coordinates, rad
};

namespace global_vars
{
    class GLOBAL_VARS_T
    {
    public:
        bool continueSim = true;
        float xPos = 0.0; // m
        float yPos = 0.0; // m
        float heading = 0.0; // rad
        const float searchRadius = 5.0; // m
        const float searchRadiusProbComputationBuffer = 0.1; // m
        const float minSearchRadius = 0.5; // m
        const double tpMax = 0.99;
        const double falsePositiveRate = 0.01; // per cell
        class RESULTS_T
        {
        public:
            float timeElapsed; // sec
            float distanceDriven; // m
            float angleTurned; // rad
            int numSamplesFound;
            int numFalseSamplesFound;
            std::vector<float> timeSamplesFound;
            RESULTS_T() // Constructor
            {
                timeElapsed = 0.0;
                distanceDriven = 0.0;
                angleTurned = 0.0;
                numSamplesFound = 0;
                numFalseSamplesFound = 0;
                timeSamplesFound.resize(7, NAN);
            }
            RESULTS_T(const RESULTS_T& other) // Copy constructor
            {
                this->timeElapsed = other.timeElapsed;
                this->distanceDriven = other.distanceDriven;
                this->angleTurned = other.angleTurned;
                this->numSamplesFound = other.numSamplesFound;
                this->numFalseSamplesFound = other.numFalseSamplesFound;
                this->timeSamplesFound = other.timeSamplesFound;
            }
        } results;
    };

    inline void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg) // angle in radians
    {
        newX = origX*cos(angleDeg)+origY*sin(angleDeg);
        newY = -origX*sin(angleDeg)+origY*cos(angleDeg);
    }
}

#endif // GLOBAL_VARS_HPP
