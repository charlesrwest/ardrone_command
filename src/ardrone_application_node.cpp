#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <unistd.h>
#include "ARDroneControllerNode.hpp"



int main(int argc, char** argv)
{
printf("Test 1\n");
printf("Initializing ROS\n");
ros::init(argc, argv, "droneTime");


std::unique_ptr<ARDroneControllerNode> myARDroneControllerNode;
try
{
printf("Initializing controller node\n");
myARDroneControllerNode.reset(new ARDroneControllerNode());
printf("Initialization completed\n");
}
catch(const std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what());
}


command commandTakeoff;
commandTakeoff.setTakeoffCommand();

printf("Adding takeoff command\n");
myARDroneControllerNode->addCommand(commandTakeoff);
printf("Takeoff command added\n");


command commandSetTargetAltitude;
commandSetTargetAltitude.setTargetAltitudeCommand(1400.0);
myARDroneControllerNode->addCommand(commandSetTargetAltitude);

command commandWaitUntilTargetAltitudeReached;
commandWaitUntilTargetAltitudeReached.setWaitUntilAltitudeReached(10.0);
myARDroneControllerNode->addCommand(commandWaitUntilTargetAltitudeReached);



command commandSetHorizontalHeading;
commandSetHorizontalHeading.setHorizontalHeadingCommand(0.025, 0.0);
myARDroneControllerNode->addCommand(commandSetHorizontalHeading);



command commandWaitUntilTagSpotted;
commandWaitUntilTagSpotted.setWaitUntilTagIsSpottedCommand(6.0);
myARDroneControllerNode->addCommand(commandWaitUntilTagSpotted);

command commandSetHorizontalHeading2;
commandSetHorizontalHeading2.setHorizontalHeadingCommand(0.00, 0.0);
myARDroneControllerNode->addCommand(commandSetHorizontalHeading2);



command commandSetHomeInOnTag;
commandSetHomeInOnTag.setHomeInOnTagCommand();
myARDroneControllerNode->addCommand(commandSetHomeInOnTag);


command commandWait;
commandWait.setWaitCommand(16.0);
myARDroneControllerNode->addCommand(commandWait);



/*
command commandLEDAnimation;
commandLEDAnimation.setLEDAnimationCommand(BLINK_STANDARD, 10, 5);
myARDroneControllerNode->addCommand(commandLEDAnimation);



command commandWait2;
commandWait2.setWaitCommand(5.0);
myARDroneControllerNode->addCommand(commandWait2);
*/

command commandLanding;
commandLanding.setLandingCommand();
printf("Adding landing command\n");
myARDroneControllerNode->addCommand(commandLanding);
printf("Landing command added\n");





while(myARDroneControllerNode->commandQueueSize() > 0)
{
}

/*
myARDroneControllerNode.reset(nullptr);
while(true)
{
}
*/

return 0;
}

