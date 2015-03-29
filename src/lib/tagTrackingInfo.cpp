#include "tagTrackingInfo.hpp"

/*
This function initializes the tag info.
@param inputXCoordinate:The standardized X coordinate of the tag (0, 1000)
@param inputYCoordinate:The standardized X coordinate of the tag (0, 1000)
@param inputWidth:The standardized width of the tag (0, 1000)
@param inputHeight:The standardized height of the tag (0, 1000)
@param inputOrientation:Tag orientation in degrees
@param inputDistance:Estimated distance to the tag in cm
*/
tagTrackingInfo::tagTrackingInfo(uint32_t inputXCoordinate, uint32_t inputYCoordinate, uint32_t inputWidth, uint32_t inputHeight, double inputOrientation, double inputDistance)
{
xCoordinate = inputXCoordinate;
yCoordinate = inputYCoordinate;
width = inputWidth;
height = inputHeight;
orientation = inputOrientation;
distance = inputDistance;
}
