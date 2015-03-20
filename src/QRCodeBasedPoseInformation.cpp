#include "QRCodeBasedPoseInformation.hpp"

/*
This function initializes the class with the first recording of the position information associated with this particular QR code.
@param inputQRCodeIndentifier: The (hopefully unique) ID string associated with the QR code
@param inputInitialPose: The 4x4 pose matrix associated with the camera in the current system
@param inputQRCodeDimension: The size in meters of one of the QR code sides
@param inputWeightedAverageConstant: How much to weight new information (estimate = oldEstimate*(1-weight) + newData*weight).  Should be 0 <= x <= 1.
@param inputMaxDelayToConsiderForWeightedAverage: How old the old estimate can be before it is simply thrown out when calculating the moving average (seconds)  

@exceptions: This function can throw exceptions
*/
QRCodeBasedPoseInformation::QRCodeBasedPoseInformation(const std::string &inputQRCodeIdentifier, const cv::Mat &inputInitialPose, double inputQRCodeDimension, double inputWeightedAverageConstant, double inputMaxDelayToConsiderForWeightedAverage)
{
//Check dimensions of pose matrix
if(inputInitialPose.dims != 2)
{
throw SOMException(std::string("Camera pose matrix is not 2D\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

if(inputInitialPose.size[0] != 4 || inputInitialPose.size[1] != 4)
{
throw SOMException(std::string("Camera pose matrix is not 4x4\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

cameraPose = inputInitialPose;
invert(cameraPose, QRCodePose); //Invert the camera pose to get the QRCode pose
associatedQRCodeIdentifier = inputQRCodeIdentifier;
associatedQRCodeDimension = inputQRCodeDimension;

//TODO: This method of matrix access needs testing.  Could be doing z,y,x instead of x,y,z
currentCameraPosition.push_back(cameraPose.at<double>(0, 3));
currentCameraPosition.push_back(cameraPose.at<double>(1, 3));
currentCameraPosition.push_back(cameraPose.at<double>(2, 3));

currentQRCodePosition.push_back(QRCodePose.at<double>(0, 3));
currentQRCodePosition.push_back(QRCodePose.at<double>(1, 3));
currentQRCodePosition.push_back(QRCodePose.at<double>(2, 3));

cameraPoseUpdateTime = std::chrono::high_resolution_clock::now();

weightedAverageConstant = inputWeightedAverageConstant;
maxDelayToBeConsideredForWeightedAverage = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputMaxDelayToConsiderForWeightedAverage));

weightedAverageCameraPosition = currentCameraPosition; 
weightedAverageQRCodePosition = currentQRCodePosition;
}

/*
This function updates the pose information using the given information.
@param inputPose: This is the 4x4 matrix representing the pose of the camera in the QR code's coordinate system
@param inputQRCodeDimension: This is the dimension of the QR code (which should be the same as when it was initialized or a exception will be thrown)

@exceptions: This function can throw exceptions
*/
void QRCodeBasedPoseInformation::updatePose(const cv::Mat &inputPose, double inputQRCodeDimension)
{
//TODO: Finish function
//Check dimensions of pose matrix
if(inputPose.dims != 2)
{
throw SOMException(std::string("Camera pose matrix is not 2D\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

if(inputPose.size[0] != 4 || inputPose.size[1] != 4)
{
throw SOMException(std::string("Camera pose matrix is not 4x4\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

if(fabs(inputQRCodeDimension - associatedQRCodeDimension) > .001)
{
throw SOMException(std::string("QR Code dimension for identifier: ") + associatedQRCodeIdentifier + std::string(" has changed.\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

cameraPose = inputPose;
invert(cameraPose, QRCodePose); //Invert the camera pose to get the QRCode pose


//TODO: This method of matrix access needs testing.  Could be doing z,y,x instead of x,y,z
currentCameraPosition[0] = cameraPose.at<double>(0, 3);
currentCameraPosition[1] = cameraPose.at<double>(1, 3);
currentCameraPosition[2] = cameraPose.at<double>(2, 3);

currentQRCodePosition[0] = QRCodePose.at<double>(0, 3);
currentQRCodePosition[1] = QRCodePose.at<double>(1, 3);
currentQRCodePosition[2] = QRCodePose.at<double>(2, 3);

//Update timestamp
auto lastUpdateTime = cameraPoseUpdateTime;
cameraPoseUpdateTime = std::chrono::high_resolution_clock::now();

//Replace weighted average with current if the last update is too stale
if((cameraPoseUpdateTime - lastUpdateTime) > maxDelayToBeConsideredForWeightedAverage)
{
weightedAverageCameraPosition = currentCameraPosition; 
weightedAverageQRCodePosition = currentQRCodePosition;
}
else
{
for(int i=0; i<3; i++)
{
weightedAverageCameraPosition[i] = weightedAverageCameraPosition[i]*(1.0-weightedAverageConstant) + currentCameraPosition[i]*weightedAverageConstant; 
weightedAverageQRCodePosition[i] = weightedAverageQRCodePosition[i]*(1.0-weightedAverageConstant) + currentQRCodePosition[i]*weightedAverageConstant;
}
}

}

/*
This function uses the camera pose matrix to map a point in the QR code's coordinate system to one in the camera's coordinate system.
@param inputPointInQRCodeSpace: an std::vector that contains the X,Y,Z coordinates in QR code space

@exceptions: This function can throw exceptions
*/
std::vector<double> QRCodeBasedPoseInformation::convertPointInQRCodeSpaceToCameraSpace(const std::vector<double> &inputPointInQRCodeSpace)
{
//Check dimensions
if(inputPointInQRCodeSpace.size() < 3)
{
throw SOMException(std::string("Point to convert has less than 3 dimensions\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

//Convert to a opencv point (4x1)
cv::Mat pointToConvert(4, 1, CV_64F);
cv::Mat buffer(4, 1, CV_64F);
pointToConvert.at<double>(0,0) = inputPointInQRCodeSpace[0];
pointToConvert.at<double>(0,1) = inputPointInQRCodeSpace[1];
pointToConvert.at<double>(0,2) = inputPointInQRCodeSpace[2];
pointToConvert.at<double>(0,3) = 1.0;


//Multiply by camera pose matrix
buffer = QRCodePose*pointToConvert;

//Convert to std::matrix
std::vector<double> pointToReturn(3);
pointToReturn[0] = buffer.at<double>(0, 0);
pointToReturn[1] = buffer.at<double>(0, 1);
pointToReturn[2] = buffer.at<double>(0, 2);



return pointToReturn;
}

/*
This function uses the QR code pose matrix to map a point in the camera's coordinate system to one in the QR code's coordinate system.
@param inputPointInQRCodeSpace: an std::vector that contains the X,Y,Z coordinates in QR code space

@exceptions: This function can throw exceptions
*/
std::vector<double> QRCodeBasedPoseInformation::convertPointInCameraSpaceToQRCodeSpace(const std::vector<double> &inputPointInCameraSpace)
{
//Check dimensions
if(inputPointInCameraSpace.size() < 3)
{
throw SOMException(std::string("Point to convert has less than 3 dimensions\n"), INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

//Convert to a opencv point (4x1)
cv::Mat pointToConvert(4, 1, CV_64F);
cv::Mat buffer(4, 1, CV_64F);
pointToConvert.at<double>(0,0) = inputPointInCameraSpace[0];
pointToConvert.at<double>(0,1) = inputPointInCameraSpace[1];
pointToConvert.at<double>(0,2) = inputPointInCameraSpace[2];
pointToConvert.at<double>(0,3) = 1.0;

//Multiply by QR code pose matrix
buffer = QRCodePose*pointToConvert;

//Convert to std::matrix
std::vector<double> pointToReturn(3);
pointToReturn[0] = buffer.at<double>(0, 0);
pointToReturn[1] = buffer.at<double>(0, 1);
pointToReturn[2] = buffer.at<double>(0, 2);

return pointToReturn;
}

