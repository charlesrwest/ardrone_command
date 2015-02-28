#include "ARDroneControllerNode.hpp"





/*
This function takes the AR drone address and starts up the nessisary ROS nodes to be able to communicate with it.  If the string is empty, it uses the default AR drone address (currently only default functionality is implemented).  If a joystick node is being published, the node will also subscribe to that and interpret its movements as commands to send to the drone (if the manual flight flag is set)
@param inputARDroneAddress: The address of the AR drone to connect to 

@exceptions: This function can throw exceptions
*/
ARDroneControllerNode::ARDroneControllerNode(std::string inputARDroneAddress)
{
spinThreadExitFlag = false;
controlEngineIsDisabled = true;

onTheGroundWithMotorsOff = true;
takingOff = false;
landing = false;
emergencyStopTriggered = false;
maintainAltitude = false;
homeInOnTag = false;
matchTagOrientation = false;
targetAltitude = 1000.0; //The altitude to maintain in mm
xHeading = 0.0; 
yHeading = 0.0; 
currentAngularVelocitySetting = 0.0; //The setting of the current velocity

//Subscribe to the legacy nav-data topic with a buffer size of 1000
legacyNavigationDataSubscriber = nodeHandle.subscribe("ardrone/navdata", 1000, &ARDroneControllerNode::handleLegacyNavData, this); 

//Create publisher to control takeoff
takeOffPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

//Create publisher to control landing
landingPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/land", 1000);

//Create publisher to control emergency stop
emergencyStopPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/reset", 1000);

//Create publisher to control heading and speed
directionAndSpeedPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

//Create client to control which camera is active
cameraControlClient = nodeHandle.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel");

//Create client to activate LED animations
setLEDAnimationClient = nodeHandle.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");

//Create client to activate flight animations
setFlightAnimationClient = nodeHandle.serviceClient<ardrone_autonomy::FlightAnim>("/ardrone/setflightanimation");

//Create client to allow flat trim calibration
calibrateFlatTrimClient = nodeHandle.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

//Create client to enable/disable USB recording
setUSBRecordingClient = nodeHandle.serviceClient<ardrone_autonomy::RecordEnable>("/ardrone/setrecord");


printf("Waiting for people to subscribe\n");

//Sleep a little while to allow subscribers to connect
while(takeOffPublisher.getNumSubscribers() < 1)
{
sleep(1);
}
printf("Takeoff publisher online\n");

while(landingPublisher.getNumSubscribers() < 1)
{
sleep(1);
}
printf("Landing publisher online\n");

while(emergencyStopPublisher.getNumSubscribers() < 1)
{
sleep(1);
}
printf("Emergency stop publisher online\n");

while(directionAndSpeedPublisher.getNumSubscribers() < 1)
{
sleep(1);
}
printf("Direction and speed publisher online\n");

printf("Subscribe wait finished\n");

sleep(1);

//Create seperate thread to call spin
SOM_TRY
spinThread.reset(new std::thread(initializeAndRunSpinThread, this));
SOM_CATCH("Error starting ROS message processing thread\n")
printf("Spin thread has been initialized\n");


//Create seperate thread to call controlDrone
SOM_TRY
controlThread.reset(new std::thread(initializeAndRunControlThread, this));
SOM_CATCH("Error starting drone control thread\n")
printf("Control thread has been initialized\n");


}

/*
This function is called by the control thread to execute the commands in the command queue.  The function normally does not return unless it receives the control shutdown command.
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::controlDrone()
{
//Handle flags
controlEngineIsDisabled = false;
SOMScopeGuard controlEngineIsDisabledGuard([&](){controlEngineIsDisabled = true;}); //Set flag to be set when the control function returns

//Process commands
command commandBuffer;
while(true)
{
//Retrieve next command
if(getNextCommand(commandBuffer) != true)
{
printf("Busy waiting!\n");
continue; //Busy wait, TODO: replace with something better 
}
printf("Doing stuff!\n");

switch(commandBuffer.type)
{
case INVALID:
continue; //Skip command
break;

case SET_CLEAR_COMMAND_QUEUE: //Clear the command queue
clearCommandQueue();
break;

case SET_EMERGENCY_LANDING_COMMAND:
landing = true;
clearCommandQueue();

SOM_TRY
enableAutoHover();//Kill any horizontal velocity
SOM_CATCH("Error enabling autohover\n")

SOM_TRY
activateLandingSequence();
SOM_CATCH("Error activating emergency landing sequence\n")
break;

case SET_EMERGENCY_STOP_COMMAND:
SOM_TRY
activateEmergencyStop();
SOM_CATCH("Error activating emergency stop\n")
emergencyStopTriggered = true;
clearCommandQueue();
break;

case SET_TAKEOFF_COMMAND:
printf("Takeoff command\n");
takingOff = true;
SOM_TRY
activateTakeoffSequence(); //TODO: control until takeoff achieved
onTheGroundWithMotorsOff = false;
SOM_CATCH("Error activating takeoff sequence")

SOM_TRY
waitUntilHoveringStateAchieved();
SOM_CATCH("Error waiting for hovering state\n")
takingOff = false;
printf("Takeoff command completed\n");
break;

case SET_LANDING_COMMAND:
printf("Landing command\n");
landing = true;
SOM_TRY
activateLandingSequence(); //TODO: control until takeoff achieved
SOM_CATCH("Error activating landing sequence")

SOM_TRY
waitUntilLandedStateAchieved();
SOM_CATCH("Error waiting for hovering state\n")
onTheGroundWithMotorsOff = true;
landing = false;
printf("Landing command completed\n");
break;

case SET_CONTROL_SHUTDOWN_COMMAND:
printf("Shutting down\n");
return; //Exit control
break;

case SET_TARGET_ALTITUDE_COMMAND:
printf("set target altitude command\n");
if(commandBuffer.doubles.size() > 0)
{
targetAltitude = commandBuffer.doubles[0];
targetAltitudeITerm = 0.0;
maintainAltitude = true; //Enable trying to meet a set altitude
}
else
{
throw SOMException("Error, set target altitude command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
printf("set target altitude command completed\n");
break;

case SET_HORIZONTAL_HEADING_COMMAND:
if(commandBuffer.doubles.size() > 1)
{
xHeading = commandBuffer.doubles[0];
yHeading = commandBuffer.doubles[1];
}
else
{
throw SOMException("Error, set set horizontal heading command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
break;

case SET_ANGULAR_VELOCITY_COMMAND:
if(commandBuffer.doubles.size() > 0)
{
currentAngularVelocitySetting = commandBuffer.doubles[0];
}
else
{
throw SOMException("Error, set angular velocity command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
break;

case SET_HOME_IN_ON_TAG_COMMAND:
xCoordinateITerm = 0.0;
yCoordinateITerm = 0.0;
homeInOnTag = true;
break;

case SET_CANCEL_HOME_IN_ON_TAG_COMMAND:
homeInOnTag = false;
break;

case SET_MATCH_ORIENTATION_TO_TAG_COMMAND:
matchTagOrientation = true;
break;

case SET_CANCEL_MATCH_ORIENTATION_TO_TAG_COMMAND:
matchTagOrientation = false;
break;

case SET_FLIGHT_ANIMATION_COMMAND:
if(commandBuffer.flightAnimations.size() > 0)
{
SOM_TRY
activateFlightAnimation(commandBuffer.flightAnimations[0]);
SOM_CATCH("Error activating flight animation\n")
}
else
{
throw SOMException("Error, set flight animation command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
break;

case SET_LED_ANIMATION_COMMAND:
if(commandBuffer.ledAnimations.size() > 0 && commandBuffer.doubles.size() > 0 && commandBuffer.integers.size() > 0)
{
SOM_TRY
activateLEDAnimationSequence(commandBuffer.ledAnimations[0], commandBuffer.doubles[0], commandBuffer.integers[0]);
SOM_CATCH("Error activating LED animation\n")
}
else
{
throw SOMException("Error, set led animation command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
break;

case SET_WAIT_COMMAND:
printf("set wait command\n");
if(commandBuffer.doubles.size() > 0)
{
SOM_TRY
waitSeconds(commandBuffer.doubles[0]);
SOM_CATCH("Error waiting for seconds\n")
}
else
{
throw SOMException("Error, waiting for seconds command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
printf("set wait command completed\n");
break;

case SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND:
if(commandBuffer.doubles.size() > 0)
{
SOM_TRY
waitSecondsOrUntilTagSpotted(commandBuffer.doubles[0]);
SOM_CATCH("Error waiting for seconds or until tag spotted\n")
}
else
{
throw SOMException("Error, waiting for seconds or until tag spotted command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}

break;

case SET_WAIT_UNTIL_ALTITUDE_REACHED:
if(commandBuffer.doubles.size() > 0 && commandBuffer.integers.size() > 0)
{
SOM_TRY
waitSecondsOrUntilAltitudeReached(commandBuffer.doubles[0], commandBuffer.integers[0]);
SOM_CATCH("Error waiting till altitude reached\n")
}
else
{
throw SOMException("Error, waiting to reach altitude command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
break;

}


}

}

/*
This function allows a command to be added to the queue of commands for the drone to execute.  This function is threadsafe, so it can be used from multiple threads but may sometimes block.
@param inputCommand: The command to add

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::addCommand(const command &inputCommand)
{
//Lock mutex so data can be accessed
SOM_TRY
commandMutex.lock();
SOM_CATCH("Error locking command mutex\n")

//Make sure mutex is unlocked when the function returns
SOMScopeGuard mutexGuard([&](){commandMutex.unlock();});

if(inputCommand.subCommands.size() > 0)
{
//This is an aggregation command, so just add the subcommands
for(int i=0; i < inputCommand.subCommands.size(); i++)
{
commandQueue.push(inputCommand.subCommands[i]);
}
return;
}

commandQueue.push(inputCommand);

printf("Added command %d\n", inputCommand.type);
}

/*
This function allows an outside thread to see how many commands there are in the current queue.  This does not count the one (if any) currently executing. This function is threadsafe (and so may sometimes block).
@return: The number of commands in the current queue

@exceptions: This function can throw exceptions
*/
int ARDroneControllerNode::commandQueueSize()
{
//Lock mutex so data can be accessed
SOM_TRY
commandMutex.lock();
SOM_CATCH("Error locking command mutex\n")

//Make sure mutex is unlocked when the function returns
SOMScopeGuard mutexGuard([&](){commandMutex.unlock();});

return commandQueue.size();
}

/*
This threadsafe function puts the next available command in the buffer and returns false if there was no command to give.  Retrieved commands are removed from the buffer.
@param inputCommandBuffer: The next command in the queue or the same as before if no command was available
@return: True if there was a new command and false otherwise
*/
bool ARDroneControllerNode::getNextCommand(command &inputCommandBuffer)
{
//Check to see if there is a command available
std::unique_lock<std::mutex> uniqueLock(commandMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking command mutex\n")

if(commandQueue.size() == 0)
{
return false;
}

//There should be a command ready and we should have mutex ownership
inputCommandBuffer = commandQueue.front();
commandQueue.pop();

return true;
}

/*
This threadsafe function removes all of the elements in the command queue
*/
void ARDroneControllerNode::clearCommandQueue()
{
//Lock mutex so data can be accessed
SOM_TRY
commandMutex.lock();
SOM_CATCH("Error locking command mutex\n")

//Make sure mutex is unlocked when the function returns
SOMScopeGuard mutexGuard([&](){commandMutex.unlock();});
while(commandQueue.size() > 0)
{
commandQueue.pop();
}
}


/*
This function sends a message to tell the drone to takeoff using its built in takeoff sequence
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::activateTakeoffSequence()
{
std_msgs::Empty emptyMessage;
takeOffPublisher.publish(emptyMessage);
}

/*
This function sends a message to tell the drone to land using its built in landing sequence
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::activateLandingSequence()
{
std_msgs::Empty emptyMessage;
landingPublisher.publish(emptyMessage);
}

/*
This function sends a message to tell the drone to active its emergency stop sequence (cutting power to the engines)
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::activateEmergencyStop()
{
std_msgs::Empty emptyMessage;
emergencyStopPublisher.publish(emptyMessage);
}

/*
This function sends a message to set the linear and angular velocity of the drone
@param inputVelocityX: The X velocity of the drone
@param inputVelocityY: The Y velocity of the drone
@param inputVelocityZ: The Z velocity of the drone
@param inputRotationZ: The Z rotation rate of the drone
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::setVelocityAndRotation(double inputVelocityX, double inputVelocityY, double inputVelocityZ, double inputRotationZ)
{
geometry_msgs::Twist twistMessage;

twistMessage.linear.x = inputVelocityX;
twistMessage.linear.y = -inputVelocityY; //Make right positive
twistMessage.linear.z = inputVelocityZ;

twistMessage.angular.x = 1.0;
twistMessage.angular.y = 1.0;
twistMessage.angular.z = inputRotationZ;

directionAndSpeedPublisher.publish(twistMessage);
}

/*
This function sets the linear and angular velocity of the drone to zero and enables the auto-hover mode to try to maintain its position
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::enableAutoHover()
{
geometry_msgs::Twist twistMessage;

twistMessage.linear.x = 0.0;
twistMessage.linear.y = 0.0;
twistMessage.linear.z = 0.0;

twistMessage.angular.x = 0.0;
twistMessage.angular.y = 0.0;
twistMessage.angular.z = 0.0;

directionAndSpeedPublisher.publish(twistMessage);
}

/*
This function either sets the active camera to the front facing one or the bottom one
@param inputSetCameraFront: True if the front camera should be set active, false if the bottom camera should be

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::setCameraFront(bool inputSetCameraFront)
{
ardrone_autonomy::CamSelect requestMessage;
if(inputSetCameraFront == true)
{
requestMessage.request.channel = 0;
}
else
{
requestMessage.request.channel = 1;
}

if(cameraControlClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/*
This function will trigger an LED animation sequence on the quadcopter
@param inputAnimationType: The type of the animation
@param inputFrequency: The frequency of the animation (if blinking), in hertz
@param inputDuration: The duration of the animation in seconds

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::activateLEDAnimationSequence(LEDAnimationType inputAnimationType, double inputFrequency, int inputDuration)
{

ardrone_autonomy::LedAnim requestMessage;

switch(inputAnimationType)
{
case BLINK_GREEN_RED:
requestMessage.request.type = 0;
break;

case BLINK_GREEN:
requestMessage.request.type = 1;
break;

case BLINK_RED:
requestMessage.request.type = 2;
break;

case BLINK_ORANGE:
requestMessage.request.type = 3;
break;

case SNAKE_GREEN_RED:
requestMessage.request.type = 4;
break;

case FIRE:
requestMessage.request.type = 5;
break;

case STANDARD:
requestMessage.request.type = 6;
break;

case RED:
requestMessage.request.type = 7;
break;

case GREEN:
requestMessage.request.type = 8;
break;

case RED_SNAKE:
requestMessage.request.type = 9;
break;

case BLANK:
requestMessage.request.type = 10;
break;

case LEFT_GREEN_RIGHT_RED:
requestMessage.request.type = 11;
break;

case LEFT_RED_RIGHT_GREEN:
requestMessage.request.type = 12;
break;

case BLINK_STANDARD:
requestMessage.request.type = 13;
break;
}

requestMessage.request.freq = inputFrequency;
requestMessage.request.duration = inputDuration;

if(setLEDAnimationClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/*
This function will trigger a flight animation
@param inputFlightAnimationType: What type of flight animation to perform

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::activateFlightAnimation(flightAnimationType inputFlightAnimationType)
{
ardrone_autonomy::FlightAnim requestMessage;

switch(inputFlightAnimationType)
{
case ARDRONE_ANIM_PHI_M30_DEG:
requestMessage.request.type = 0;
break;

case ARDRONE_ANIM_PHI_30_DEG:
requestMessage.request.type = 1;
break;

case ARDRONE_ANIM_THETA_M30_DEG:
requestMessage.request.type = 2;
break;

case ARDRONE_ANIM_THETA_30_DEG:
requestMessage.request.type = 3;
break;

case ARDRONE_ANIM_THETA_20DEG_YAW_200DEG:
requestMessage.request.type = 4;
break;

case ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG:
requestMessage.request.type = 5;
break;

case ARDRONE_ANIM_TURNAROUND:
requestMessage.request.type = 6;
break;

case ARDRONE_ANIM_TURNAROUND_GODOWN:
requestMessage.request.type = 7;
break;

case ARDRONE_ANIM_YAW_SHAKE:
requestMessage.request.type = 8;
break;

case ARDRONE_ANIM_YAW_DANCE:
requestMessage.request.type = 9;
break;

case ARDRONE_ANIM_PHI_DANCE:
requestMessage.request.type = 10;
break;

case ARDRONE_ANIM_THETA_DANCE:
requestMessage.request.type = 11;
break;

case ARDRONE_ANIM_VZ_DANCE:
requestMessage.request.type = 12;
break;

case ARDRONE_ANIM_WAVE:
requestMessage.request.type = 13;
break;

case ARDRONE_ANIM_PHI_THETA_MIXED:
requestMessage.request.type = 14;
break;

case ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED:
requestMessage.request.type = 15;
break;

case ARDRONE_ANIM_FLIP_AHEAD:
requestMessage.request.type = 16;
break;

case ARDRONE_ANIM_FLIP_BEHIND:
requestMessage.request.type = 17;
break;

case ARDRONE_ANIM_FLIP_LEFT:
requestMessage.request.type = 18;
break;

case ARDRONE_ANIM_FLIP_RIGHT:
requestMessage.request.type = 19;
break;

}

requestMessage.request.duration = 0;

if(setFlightAnimationClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/*
This function causes the drone to recalibrate its rotations using the assumption that it is on a flat service.  Don't use it when it is not on a flat surface.
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::calibrateFlatTrim()
{
std_srvs::Empty requestMessage;


if(calibrateFlatTrimClient.call(requestMessage))
{
//Request succeeded
}
else
{
}
}

/*
This function causes the drone to start recording to its USB stick (if it has one) or stop recording.
@param inputStartRecording: Set to true if the drone should start recording and false if it should stop
@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::enableUSBRecording(bool inputStartRecording)
{
ardrone_autonomy::RecordEnable requestMessage;


requestMessage.request.enable = inputStartRecording;

if(setUSBRecordingClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/*
This function is used as a callback to handle legacy nav-data.
@param inputMessage: The legacy nav-data message to handle 
*/
void ARDroneControllerNode::handleLegacyNavData(const ardrone_autonomy::Navdata::ConstPtr &inputMessage)
{
//Update navdata cache
try
{
//Lock mutex so data can be accessed
std::unique_lock<std::mutex> uniqueLock(navDataMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking command mutex\n")

//printf("Updating navdata\n");

batteryPercent = inputMessage->batteryPercent;

switch(inputMessage->state)
{
case 0:
state = UNKNOWN_STATE;
break;

case 1:
state = INITIALIZED;
break;

case 2:
state = LANDED;
break;

case 3:
case 7:
state = FLYING;
break;

case 4:
state = HOVERING;
break;

case 5:
state = TEST;
break;

case 6:
state = TAKING_OFF;
break;

case 8:
state = LANDING;
break;

case 9:
state = LOOPING;
break;
}

rotationX = inputMessage->rotX;
rotationY = inputMessage->rotY;
rotationZ = inputMessage->rotZ;
magneticX = inputMessage->magX;
magneticY = inputMessage->magY;
magneticZ = inputMessage->magZ;
pressure  = inputMessage->pressure;
temperature = inputMessage->temp;
windSpeed = inputMessage->wind_speed;
windAngle = inputMessage->wind_angle;
windCompensationAngle = inputMessage->wind_comp_angle;
altitude  = inputMessage->altd;
motorPWM[0] = inputMessage->motor1;
motorPWM[1] = inputMessage->motor2;
motorPWM[2] = inputMessage->motor3;
motorPWM[3] = inputMessage->motor4;
velocityX = inputMessage->vx;
velocityY = inputMessage->vy;
velocityZ = inputMessage->vz;
accelerationX = inputMessage->ax;
accelerationY = inputMessage->ay;
accelerationZ = inputMessage->az;

//printf("I see %d tags\n", inputMessage->tags_count);

//Set tag info
trackedTags.clear();
for(int i=0; i < inputMessage->tags_count; i++)
{
trackedTags.push_back(tagTrackingInfo(inputMessage->tags_xc[i], inputMessage->tags_yc[i], inputMessage->tags_width[i], inputMessage->tags_height[i], inputMessage->tags_orientation[i], inputMessage->tags_distance[i]));
}

//Release mutex
uniqueLock.unlock();

//Signal data is available
newNavDataAvailable.notify_all();
}
catch(std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what()); 
}
}


/*
This function cleans up the object and waits for any threads the object spawned to return.
*/
ARDroneControllerNode::~ARDroneControllerNode()
{
//activateLandingSequence();



//Send shutdown command to make control function return
command shutdownCommand;
shutdownCommand.setControlShutdownCommand();

try
{
addCommand(shutdownCommand);
}
catch(const std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what());
}

controlThread->join();
printf("Control thread joined\n");

spinThreadExitFlag = true;
spinThread->join();
printf("Spin thread joined\n");
}

/*
This function waits on the given locked mutex until the signal for the next navdata update occurs (though there can be fake wakeups)
@param inputLockedUniqueLock: A locked unique lock that has the navdata mutex.  It is returned in a locked state again

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::waitUntilNextNavDataUpdate(std::unique_lock<std::mutex> &inputLockedUniqueLock)
{
newNavDataAvailable.wait(inputLockedUniqueLock);
}

/*
This function blocks until the ARDrone has achieved the hovering state.  This is normally used after the takeoff signal has been sent to wait until the drone finishes taking off.  It doesn't update low level behaviors.

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::waitUntilHoveringStateAchieved()
{
//Lock mutex so data can be accessed
std::unique_lock<std::mutex> uniqueLock(navDataMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking navdata mutex\n")

while(true)
{
if(state == HOVERING || state == FLYING)
{
break;
}
waitUntilNextNavDataUpdate(uniqueLock); //Wait until next update
}

}

/*
This function blocks until the ARDrone has achieved the landed state.  This is normally used after the land signal has been sent to wait until the drone finishes setting down.  It updates low level behaviors.

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::waitUntilLandedStateAchieved()
{
//Lock mutex so data can be accessed
std::unique_lock<std::mutex> uniqueLock(navDataMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking navdata mutex\n")

while(true)
{
if(state == LANDED)
{
break;
}

SOM_TRY
handleLowLevelBehavior(uniqueLock);
SOM_CATCH("Error handling low level behavior\n")

SOM_TRY
waitUntilNextNavDataUpdate(uniqueLock); //Wait until next update
SOM_CATCH("Error waiting for next nav update\n")
}

}

/*
This function blocks until the given amount of time has passed, while maintaining low level behaviors (tag tracking and altitude control).
@param inputSecondsToWait: The number of seconds to wait

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::waitSeconds(double inputSecondsToWait)
{
//Get the start time
auto originalTime = std::chrono::high_resolution_clock::now();
//Lock mutex so data can be accessed
std::unique_lock<std::mutex> uniqueLock(navDataMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking navdata mutex\n")

while(true)
{
auto currentTime = std::chrono::high_resolution_clock::now();
auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - originalTime);

if(elapsedTime.count() >= inputSecondsToWait*1000.0)
{
break; //Timer finished
}

//Handle low level behaviors
SOM_TRY
handleLowLevelBehavior(uniqueLock);
SOM_CATCH("Error handling low level behavior\n")

SOM_TRY
waitUntilNextNavDataUpdate(uniqueLock); //Wait until next update
SOM_CATCH("Error waiting for next nav update\n")
}

}

/*
This function blocks until the given amount of time has passed, while maintaining low level behaviors (tag tracking and altitude control).
@param inputSecondsToWait: The number of seconds to wait

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::waitSecondsOrUntilTagSpotted(double inputSecondsToWait)
{
//Get the start time
auto originalTime = std::chrono::high_resolution_clock::now();
//Lock mutex so data can be accessed
std::unique_lock<std::mutex> uniqueLock(navDataMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking navdata mutex\n")

while(true)
{
auto currentTime = std::chrono::high_resolution_clock::now();
auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - originalTime);

if(elapsedTime.count() >= inputSecondsToWait*1000.0 || trackedTags.size() > 0)
{
break; //Timer finished
}

//Handle low level behaviors
SOM_TRY
handleLowLevelBehavior(uniqueLock);
SOM_CATCH("Error handling low level behavior\n")

SOM_TRY
waitUntilNextNavDataUpdate(uniqueLock); //Wait until next update
SOM_CATCH("Error waiting for next nav update\n")
}
}

/*
This function blocks until the given amount of time has passed or the target altitude is reached, while maintaining low level behaviors (tag tracking and altitude control).
@param inputNumberOfSeconds: The number of seconds to wait
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 1 mm.

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::waitSecondsOrUntilAltitudeReached(double inputNumberOfSeconds, int inputNumberOfMillimetersToTarget)
{
//Get the start time
auto originalTime = std::chrono::high_resolution_clock::now();
//Lock mutex so data can be accessed
std::unique_lock<std::mutex> uniqueLock(navDataMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking navdata mutex\n")

while(true)
{
auto currentTime = std::chrono::high_resolution_clock::now();
auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - originalTime);

if(elapsedTime.count() >= inputNumberOfSeconds*1000.0 || (fabs(altitude - targetAltitude) < inputNumberOfMillimetersToTarget) )
{
break; //Timer finished
}

//Handle low level behaviors
SOM_TRY
handleLowLevelBehavior(uniqueLock);
SOM_CATCH("Error handling low level behavior\n")

SOM_TRY
waitUntilNextNavDataUpdate(uniqueLock); //Wait until next update
SOM_CATCH("Error waiting for next nav update\n")
}
}


/*
This function takes care of low level behavior that depends on the specific state variables (such as reaching the desired altitude, orientation, and following tags).  It assumes that the navdata mutex is locked by the function that called it, so that it can access navdata freely.  This function is typically called in waiting functions after each navdata update. 
@param inputLockedUniqueLock: A locked unique lock that has the navdata mutex.  It is returned in a locked state again

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::handleLowLevelBehavior(std::unique_lock<std::mutex> &inputLockedUniqueLock)
{

if(controlEngineIsDisabled || onTheGroundWithMotorsOff || emergencyStopTriggered)
{
printf("No control %d %d %d\n", controlEngineIsDisabled,  onTheGroundWithMotorsOff, emergencyStopTriggered);
return; //Return if we shouldn't be trying to fly
}


//printf("Hello!\n");

double zThrottle = 0.0;
if(maintainAltitude)
{
double pTerm = (targetAltitude - fabs(altitude));
targetAltitudeITerm = targetAltitudeITerm + pTerm;

zThrottle = pTerm/600.0 + targetAltitudeITerm/100000000.0; //Simple P control for now

/*
if(zThrottle > 0.9) //Clamp values
{
zThrottle = .9;
}
if(zThrottle < -0.9)
{
zThrottle = -0.9;
}
*/


//printf("Altitude values: target: %.1lf Current: %.1lf Diff: %.1lf throt:  %.1lf I:%.1lf\n", targetAltitude, altitude, targetAltitude -altitude, zThrottle, targetAltitudeITerm); 
}

if(trackedTags.size() > 0)
{
//printf("I see %ld tags\n", trackedTags.size());
double xCoordinate = trackedTags[0].xCoordinate;
double yCoordinate = trackedTags[0].yCoordinate;
printf("XCoordinate: %.2lf  YCoordinate: %.2lf\n", xCoordinate, yCoordinate);
}

double xThrottle = xHeading;
double yThrottle = yHeading;
if(homeInOnTag && trackedTags.size() > 0)
{
//Override heading values if there is a tag to follow
//(xCoordinate maxs at +-1000, so throttle should max at +-.1)
double xCoordinate = trackedTags[0].xCoordinate-500.0;
double yCoordinate = -(trackedTags[0].yCoordinate-500.0);
xCoordinateITerm = xCoordinateITerm + xCoordinate/10000000.0;
yCoordinateITerm = yCoordinateITerm + yCoordinate/10000000.0;

printf("XCoordinate: %.2lf  YCoordinate: %.2lf XCoordinateI: %.2lf  YCoordinateI: %.2lf\n\n", xCoordinate, yCoordinate, xCoordinateITerm, yCoordinateITerm);

//X: Tag is right = high positive
//Y: Tag is back = high positive

if(xCoordinate > 1000)
{
xCoordinate = 1000;
}
if(xCoordinate < -1000)
{
xCoordinate = -1000;
}

if(yCoordinate > 1000)
{
yCoordinate = 1000;
}
if(yCoordinate < -1000)
{
yCoordinate = -1000;
}



xThrottle = yCoordinate/3500.0;// + yCoordinateITerm/500.0; //Simple P control for now
//(yCoordinate maxs at +-1000, so throttle should max at +-.1)
yThrottle = xCoordinate/3500.0; //+ xCoordinateITerm/500.0; //Simple P control for now
//yThrottle = 0;
printf("Applying x:%.2lf y:%.2lf\n", xThrottle, yThrottle);
}

double zRotationThrottle = currentAngularVelocitySetting;
if(matchTagOrientation)
{
//Override angular velocity setting to track tag orientation
zRotationThrottle = trackedTags[0].orientation / 180; //Might need offset and other configuration
}


//Unlock while sending ROS message
inputLockedUniqueLock.unlock();

//Tell the drone how it should move
setVelocityAndRotation(xThrottle, yThrottle, zThrottle, zRotationThrottle);

SOM_TRY
inputLockedUniqueLock.lock();
SOM_CATCH("Error relocking mutex\n")
}



/*
This function repeatedly calls ros::spinOnce until the spinThreadExitFlag in the given object is set (usually by the object destructor.  It is usually run in a seperate thread.
@param inputARDroneControllerNode: The node this function call is associated
*/
void initializeAndRunSpinThread(ARDroneControllerNode *inputARDroneControllerNode)
{
while(inputARDroneControllerNode->spinThreadExitFlag != true)
{
ros::spinOnce();
}
}

/*
This function calls controlDrone until the controlThreadExitFlag in the given object is set (usually by the object destructor.  It is usually run in a seperate thread.
@param inputARDroneControllerNode: The node this function call is associated
*/
void initializeAndRunControlThread(ARDroneControllerNode *inputARDroneControllerNode)
{
try
{
//Set camera to look down for tags
inputARDroneControllerNode->setCameraFront(false);
inputARDroneControllerNode->controlDrone();
}
catch(const std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what());
}
}
