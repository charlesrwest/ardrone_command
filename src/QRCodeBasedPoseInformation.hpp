#ifndef QRCODEBASEDPOSEINFORMATIONHPP
#define QRCODEBASEDPOSEINFORMATIONHPP

#include<vector>
#include<map>
#include<string>
#include<chrono>
#include "QRCodeStateEstimator.hpp" 


/*
This class stores the information associated with the camera's pose relative to a single QR code based identifier (one label).  It is meant to help use the information and use updates to help intelligently estimate the camera's position in the QR code coordinate system.
*/
class QRCodeBasedPoseInformation
{
public:
/*
This function initializes the class with the first recording of the position information associated with this particular QR code.
@param inputQRCodeIndentifier: The (hopefully unique) ID string associated with the QR code
@param inputInitialPose: The 4x4 pose matrix associated with the camera in the current system
@param inputQRCodeDimension: The size in meters of one of the QR code sides
@param inputWeightedAverageConstant: How much to weight new information (estimate = oldEstimate*(1-weight) + newData*weight).  Should be 0 <= x <= 1.
@param inputMaxDelayToConsiderForWeightedAverage: How old the old estimate can be before it is simply thrown out when calculating the moving average (seconds)  

@exceptions: This function can throw exceptions
*/
QRCodeBasedPoseInformation(const std::string &inputQRCodeIdentifier, const cv::Mat &inputInitialPose, double inputQRCodeDimension, double inputWeightedAverageConstant = .2, double inputMaxDelayToConsiderForWeightedAverage = .5);

/*
This function updates the pose information using the given information.
@param inputPose: This is the 4x4 matrix representing the pose of the camera in the QR code's coordinate system
@param inputQRCodeDimension: This is the dimension of the QR code (which should be the same as when it was initialized or a exception will be thrown)

@exceptions: This function can throw exceptions
*/
void updatePose(const cv::Mat &inputPose, double inputQRCodeDimension);

/*
This function uses the camera pose matrix to map a point in the QR code's coordinate system to one in the camera's coordinate system.
@param inputPointInQRCodeSpace: an std::vector that contains the X,Y,Z coordinates in QR code space

@exceptions: This function can throw exceptions
*/
std::vector<double> convertPointInQRCodeSpaceToCameraSpace(const std::vector<double> &inputPointInQRCodeSpace);

/*
This function uses the QR code pose matrix to map a point in the camera's coordinate system to one in the QR code's coordinate system.
@param inputPointInQRCodeSpace: an std::vector that contains the X,Y,Z coordinates in QR code space

@exceptions: This function can throw exceptions
*/
std::vector<double> convertPointInCameraSpaceToQRCodeSpace(const std::vector<double> &inputPointInCameraSpace);

cv::Mat cameraPose; //4x4 opencv pose matrix indicating the position and orientation of the camera in the QR code's coordinate system
cv::Mat QRCodePose; //4x4 opencv pose matrix indicating the position and orientation of the QR code in the camera's coordinate system
std::string associatedQRCodeIdentifier;
double associatedQRCodeDimension;
std::vector<double> currentCameraPosition; //X Y Z coordinates of camera in QR code coordinate system
std::vector<double> currentQRCodePosition; //X Y Z coordinates of QR code in the camera coordinate system
std::chrono::time_point<std::chrono::high_resolution_clock> cameraPoseUpdateTime; //The timestamp of when the last camera pose update was received

double weightedAverageConstant;
std::chrono::seconds maxDelayToBeConsideredForWeightedAverage;
std::vector<double> weightedAverageCameraPosition; //X Y Z coordinates of camera in QR code coordinate system
std::vector<double> weightedAverageQRCodePosition; //X Y Z coordinates of QR code in the camera coordinate system
};




#endif 
