#ifndef TAGTRACKINGINFOHPP
#define TAGTRACKINGINFOHPP

#include<cstdint>

/*
This class is just a container to make working with tag tracking data easier.  For now we are just going to assume all detected tags are oriented roundels.
*/
class tagTrackingInfo
{
public:
/*
This function initializes the tag info.
@param inputXCoordinate:The standardized X coordinate of the tag (0, 1000)
@param inputYCoordinate:The standardized X coordinate of the tag (0, 1000)
@param inputWidth:The standardized width of the tag (0, 1000)
@param inputHeight:The standardized height of the tag (0, 1000)
@param inputOrientation:Tag orientation in degrees
@param inputDistance:Estimated distance to the tag in cm
*/
tagTrackingInfo(uint32_t inputXCoordinate, uint32_t inputYCoordinate, uint32_t inputWidth, uint32_t inputHeight, double inputOrientation, double inputDistance);

uint32_t xCoordinate; //The standardized X coordinate of the tag (0, 1000)
uint32_t yCoordinate; //The standardized X coordinate of the tag (0, 1000)
uint32_t width;       //The standardized width of the tag (0, 1000)
uint32_t height;      //The standardized height of the tag (0, 1000)
double   orientation; //Tag orientation in degrees
double   distance;    //Estimated distance to the tag in cm
};

#endif
