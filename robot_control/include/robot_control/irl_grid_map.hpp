#ifndef IRL_GRID_MAP_HPP
#define IRL_GRID_MAP_HPP
#include <math.h>
#include <cstdlib>
#include <stdexcept>
#include <string>

template<class T>
class IRLGridMap
{
public:
    // Members
    size_t xSize;
    size_t ySize;
    // For data logging
    float robotXPos; // m
    float robotYPos; // m
    float robotHeading; // rad
    double elapsedTime; // sec
    int32_t numSamplesFound;
    int32_t numFalseSamplesFound;
    double timeWhenSamplesFound[7] = {NAN,NAN,NAN,NAN,NAN,NAN,NAN};
    // Methods
	IRLGridMap(float res, float xDim, float yDim);
	T& atPos(float xPos, float yPos);
	T& atIndex(size_t xIndex, size_t yIndex);
    void getPos(float& xPos, float& yPos, size_t xIndex, size_t yIndex);
    void getIndex(size_t& xIndex, size_t& yIndex, float xPos, float yPos);
    
    //get total number of bytes which represents object and array data
    size_t getTotalObjectSizeBytes()
    {
        return sizeof(IRLGridMap<T>)+sizeof(T)*xSize*ySize;
    }
    
    //copy all object data then map data to buffer with size from getTotalObjectSizeBytes()
    void serializeObject(char *buf, IRLGridMap<T> &map)
    {
        std::copy((char*)&map,(char*)&map+sizeof(map),buf);
        std::copy((char*)array_,(char*)array_+xSize*ySize*sizeof(T),buf+sizeof(map));
    }
    
    //this will not own the new object pointer. malloc in mem. reinit object from buf
    static IRLGridMap<T> * unserializeObject(char *buf)
    {
        IRLGridMap<T> *newMap=(IRLGridMap<T>*)buf;
        newMap->array_=(T*)(buf+sizeof(IRLGridMap<T>));
        newMap->mapFromUnserialize=true;
        return newMap;
    }
    
    
	~IRLGridMap();
private:
    // Members
    T (*array_);
    std::string exceptionString_;
    float mapRes_;
    bool mapFromUnserialize;
    // Methods
    void throwOutOfBoundsException_(size_t xIndex, size_t yIndex);
};

template<class T>
IRLGridMap<T>::IRLGridMap(float res, float xDim, float yDim)
{
    mapRes_ = res;
    xSize = (size_t)ceil(xDim/res);
    ySize = (size_t)ceil(yDim/res);
    array_ = (T*) std::malloc(xSize*ySize*sizeof(T));
    mapFromUnserialize=false;
}

template<class T>
T& IRLGridMap<T>::atPos(float xPos, float yPos)
{
    size_t xIndex = (size_t)ceil(xPos/mapRes_) - 1;
    size_t yIndex = (size_t)ceil(yPos/mapRes_) - 1;
	return atIndex(xIndex, yIndex);
}

template<class T>
T& IRLGridMap<T>::atIndex(size_t xIndex, size_t yIndex)
{
    if(xIndex<xSize && yIndex<ySize)
	{
        return array_[xIndex*ySize+yIndex];
	}
    else
    {
        throwOutOfBoundsException_(xIndex, yIndex);
    }
}

template<class T>
void IRLGridMap<T>::getPos(float &xPos, float &yPos, size_t xIndex, size_t yIndex)
{
    if(xIndex<xSize && yIndex<ySize)
    {
        xPos = xIndex*mapRes_ + mapRes_/2.0;
        yPos = yIndex*mapRes_ + mapRes_/2.0;
    }
    else
    {
        throwOutOfBoundsException_(xIndex, yIndex);
    }
}

template<class T>
void IRLGridMap<T>::getIndex(size_t &xIndex, size_t &yIndex, float xPos, float yPos)
{
    xIndex = (size_t)ceil(xPos/mapRes_) - 1;
    yIndex = (size_t)ceil(yPos/mapRes_) - 1;
    if(xIndex>=xSize || yIndex>=ySize)
    {
        throwOutOfBoundsException_(xIndex, yIndex);
        xIndex = 0;
        yIndex = 0;
    }
}

template<class T>
IRLGridMap<T>::~IRLGridMap()
{
//    for(int i=0; i<xSize; i++)
//    {
//         delete[] array_[i];
//    }
//	delete[] array_;
    if(!mapFromUnserialize)
    {
        delete array_;
    }
}

template<class T>
void IRLGridMap<T>::throwOutOfBoundsException_(size_t xIndex, size_t yIndex)
{
    exceptionString_ = "IRLGridMap: tried to access outside array bounds. Index [" + std::to_string(xIndex) + "," + std::to_string(yIndex) + "]";
    throw std::out_of_range(exceptionString_.c_str());
}

#endif // IRL_GRID_MAP_HPP
