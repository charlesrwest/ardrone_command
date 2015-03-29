#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <unistd.h>
#include "ARDroneControllerNode.hpp"
#include "ardrone_command/serialized_ardrone_command.h"
#include "ardrone_command/commandInterface.h"

/*
This function is used to provide a service which allows other processes on the network to send commands to the AR Drone controlled by this application.  The function will send a response of "true" if the command was successfully added to the queue and "false" otherwise.
@param inputRequest: The request, which contains a serialized command for the AR drone
@param inputResponse: The buffer for the function to place the response in
@return: Always true

@exceptions: This function can throw exceptions.
*/
bool commandReceivedFromNetwork(ardrone_command::commandInterface::Request &inputRequest, ardrone_command::commandInterface::Response &inputResponse);

//Initialize an object for this to point to before initializing the add command service.
static std::unique_ptr<ARDroneControllerNode> myARDroneControllerNode;


int main(int argc, char** argv)
{


ros::init(argc, argv, "ardrone_command");
ros::NodeHandle nodeHandle;


//Define image size
//int imageWidth = 1280;
//int imageHeight = 720;
int imageWidth = 640;
int imageHeight = 360;

//Define camera matrix according to webcam calibration (from OpenCV camera calibration file)
//Scaled values due to the ARDrone's resolution change (from ardrone_autonomy having bad default without option to change)
cv::Mat_<double> cameraMatrix(3, 3);
cameraMatrix.at<double>(0,0) =  1.1485540667575478e+03*.5;
cameraMatrix.at<double>(0,1) = 0.;
cameraMatrix.at<double>(0,2) = 6.3950000000000000e+02*.5;
cameraMatrix.at<double>(1,0) =  0.;
cameraMatrix.at<double>(1,1) = 1.1485540667575478e+03*.5;
cameraMatrix.at<double>(1,2) = 3.5950000000000000e+02*.5;
cameraMatrix.at<double>(2,0) = 0.0;
cameraMatrix.at<double>(2,1) = 0.0; 
cameraMatrix.at<double>(2,2) = 1.0;
    

//Define distortion parameters according to webcam calibration (from OpenCV camera calibration file)
cv::Mat_<double> distortionParameters(1, 5); //k1, k2, p1, p2, k3
distortionParameters.at<double>(0, 0) = -5.2981696528785138e-01;
distortionParameters.at<double>(0, 1) = 3.3890562816386838e-01;
distortionParameters.at<double>(0, 2) =  0.0;
distortionParameters.at<double>(0, 3) =  0.0;
distortionParameters.at<double>(0, 4) = -1.1542908311868905e-01;



try
{
printf("Initializing controller node\n");
myARDroneControllerNode.reset(new ARDroneControllerNode(imageWidth, imageHeight, cameraMatrix, distortionParameters));
printf("Initialization completed\n");
}
catch(const std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what());
}

/*
//Provide service to take commands
auto serverHandle = nodeHandle.advertiseService("/ardrone_command/commandInterface", commandReceivedFromNetwork);

while(true)
{
printf("Yawn!\n");
//ros::spin();
sleep(10); //Sleep while callbacks handle everything
}
*/

command commandTakeoff;
commandTakeoff.setTakeoffCommand();

printf("Adding takeoff command\n");
myARDroneControllerNode->addCommand(commandTakeoff);
printf("Takeoff command added\n");


command commandSetTargetAltitude;
commandSetTargetAltitude.setTargetAltitudeCommand(500.0);
myARDroneControllerNode->addCommand(commandSetTargetAltitude);

command commandWaitUntilTargetAltitudeReached;
commandWaitUntilTargetAltitudeReached.setWaitUntilAltitudeReached(10.0);
myARDroneControllerNode->addCommand(commandWaitUntilTargetAltitudeReached);

command commandWaitForQRCode;
commandWaitForQRCode.setWaitUntilSpecificQRCodeIsSpottedCommand("BigQRCode", 3.0);
myARDroneControllerNode->addCommand(commandWaitForQRCode);

command commandLookAtQRCodePoint;
commandLookAtQRCodePoint.setMaintainOrientationTowardSpecificQRCode("BigQRCode");
myARDroneControllerNode->addCommand(commandLookAtQRCodePoint);

command commandGoToQRCodePoint;
commandGoToQRCodePoint.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", .5, 0.0, 3.0+.5);
myARDroneControllerNode->addCommand(commandGoToQRCodePoint);


command QRCodePointWait1; QRCodePointWait1.setWaitUntilPositionAtSpecificQRCodePointReachedCommand(10.0);
myARDroneControllerNode->addCommand(QRCodePointWait1);

command commandGoToQRCodePoint2;
commandGoToQRCodePoint2.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", .5, 0.0, 3.0-.5);
myARDroneControllerNode->addCommand(commandGoToQRCodePoint2);

myARDroneControllerNode->addCommand(QRCodePointWait1);


command commandGoToQRCodePoint3;
commandGoToQRCodePoint3.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", -.5, 0.0, 3.0-.5);
myARDroneControllerNode->addCommand(commandGoToQRCodePoint3);

myARDroneControllerNode->addCommand(QRCodePointWait1);


command commandGoToQRCodePoint4;
commandGoToQRCodePoint4.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", -.5, 0.0, 3.0+.5);
myARDroneControllerNode->addCommand(commandGoToQRCodePoint4);

myARDroneControllerNode->addCommand(QRCodePointWait1);


command commandGoToQRCodePoint5;
commandGoToQRCodePoint5.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", .5, 0.0, 3.0+.5);
myARDroneControllerNode->addCommand(commandGoToQRCodePoint5);

myARDroneControllerNode->addCommand(QRCodePointWait1);


command commandWait;
commandWait.setWaitCommand(4.0);
myARDroneControllerNode->addCommand(commandWait);


/*
command commandWait;
commandWait.setWaitCommand(500.0);
myARDroneControllerNode->addCommand(commandWait);



command commandWaitForQRCode;
commandWaitForQRCode.setWaitUntilSpecificQRCodeIsSpottedCommand("BigQRCode", 3.0);
myARDroneControllerNode->addCommand(commandWaitForQRCode);
*/


/*
command commandLookAtQRCodePoint;
commandLookAtQRCodePoint.setMaintainOrientationTowardSpecificQRCode("BigQRCode");
myARDroneControllerNode->addCommand(commandLookAtQRCodePoint);
*/


//Go to -1.0, 0 4.0
/*
command commandGoToQRCodePoint;
commandGoToQRCodePoint.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", 0.0, -0.5, 4.0);
myARDroneControllerNode->addCommand(commandGoToQRCodePoint);
*/

/*
command commandSetHorizontalHeading;
commandSetHorizontalHeading.setHorizontalHeadingCommand(0.05, 0.0);
myARDroneControllerNode->addCommand(commandSetHorizontalHeading);
*/
/*
command commandWait;
commandWait.setWaitCommand(2.0);
myARDroneControllerNode->addCommand(commandWait);
*/


while(myARDroneControllerNode->commandQueueSize() > 0)
{
}


ros::shutdown();

return 0;
}

/*
This function is used to provide a service which allows other processes on the network to send commands to the AR Drone controlled by this application.  The function will send a response of "true" if the command was successfully added to the queue and "false" otherwise.
@param inputRequest: The request, which contains a serialized command for the AR drone
@param inputResponse: The buffer for the function to place the response in
@return: Always true

@exceptions: This function can throw exceptions.
*/
bool commandReceivedFromNetwork(ardrone_command::commandInterface::Request &inputRequest, ardrone_command::commandInterface::Response &inputResponse)
{
auto receivedCommands = deserialize_commands(inputRequest.command);

printf("Received command\n");

for(int i=0; i<receivedCommands.size(); i++)
{
SOM_TRY
myARDroneControllerNode->addCommand(receivedCommands[i]);
SOM_CATCH("Error, could not add command\n")
}

inputResponse.received = true;

return true;
}




