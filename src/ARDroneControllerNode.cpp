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
shutdownControlEngine = false;
currentlyWaiting = false;

onTheGroundWithMotorsOff = true;
takingOff = false;
landing = false;
emergencyStopTriggered = false;
maintainAltitude = false;
targetAltitude = 1000.0; //The altitude to maintain in mm
xHeading = 0.0; 
yHeading = 0.0; 
currentAngularVelocitySetting = 0.0; //The setting of the current velocity

//Subscribe to the legacy nav-data topic with a buffer size of 1000
navDataSubscriber = nodeHandle.subscribe("ardrone/navdata", 1000, &ARDroneControllerNode::handleNavData, this); 

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

//Unwrap command so that it can be easily processed
std::vector<command> simpleCommandsBuffer;
unwrapCommand(inputCommand, simpleCommandsBuffer);

for(int i=0; i<simpleCommandsBuffer.size(); i++)
{
commandQueue.push(simpleCommandsBuffer[i]);
}

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
This threadsafe function copies the next available command in the buffer and returns false if there was no command to give.  Retrieved commands are remain in the buffer.
@param inputCommandBuffer: The next command in the queue or the same as before if no command was available
@return: True if there was a new command and false otherwise
*/
bool ARDroneControllerNode::copyNextCommand(command &inputCommandBuffer)
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

return true;
}

/*
This threadsafe function deletes the next available command in the buffer.  If the buffer is empty, it returns without deleting.
*/
void ARDroneControllerNode::popNextCommand()
{
//Check to see if there is a command available
std::unique_lock<std::mutex> uniqueLock(commandMutex, std::defer_lock);
SOM_TRY
uniqueLock.lock();
SOM_CATCH("Error locking command mutex\n")

if(commandQueue.size() == 0)
{
return;
}

//There should be a command ready and we should have mutex ownership
commandQueue.pop();
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
This function is called to update the local cache of navdata information based on the given message.
@param inputMessage: The navdata message to update the cache with
*/
void ARDroneControllerNode::updateNavdataCache(const ardrone_autonomy::Navdata::ConstPtr &inputMessage)
{
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

//Set tag info
trackedTags.clear();
for(int i=0; i < inputMessage->tags_count; i++)
{
trackedTags.push_back(tagTrackingInfo(inputMessage->tags_xc[i], inputMessage->tags_yc[i], inputMessage->tags_width[i], inputMessage->tags_height[i], inputMessage->tags_orientation[i], inputMessage->tags_distance[i]));
}

//Set when the update was received
navdataUpdateTime = std::chrono::high_resolution_clock::now();
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

requestMessage.request.type = (int) inputAnimationType;

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

requestMessage.request.type = (int) inputFlightAnimationType;
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
This function is used as a callback to handle nav-data.
@param inputMessage: The  nav-data message to handle 
*/
void ARDroneControllerNode::handleNavData(const ardrone_autonomy::Navdata::ConstPtr &inputMessage)
{
//Update navdata cache
updateNavdataCache(inputMessage);

//Send control messages for this cycle and process commands
SOM_TRY
processCurrentCommandsForUpdateCycle();
SOM_CATCH("Error adjusting/sending commands for commands/behavior")

}

/*
This function is used as a callback to handle images from the AR drone.
@param inputImageMessage: The image message to handle 
*/
void ARDroneControllerNode::handleImageUpdate(const sensor_msgs::ImageConstPtr& inputImageMessage)
{
//TODO: Implement function
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

spinThreadExitFlag = true;
spinThread->join();
printf("Spin thread joined\n");
}



/*
This function returns true if the ARDrone has achieved the hovering state.
@return: True if the hovering state has been achieved
*/
bool ARDroneControllerNode::checkIfHoveringStateAchieved()
{
return state == HOVERING || state == FLYING;
}

/*
This function returns if the ARDrone has achieved the landed state.
@return: True if the landed state has been achieved
*/
bool ARDroneControllerNode::checkIfLandedStateAchieved()
{
return state == LANDED;
}

/*
This function checks if a tag has been spotted.
@return: True if there is one or more tags in the navdata cache
*/
bool ARDroneControllerNode::checkIfTagSpotted()
{
return trackedTags.size() > 0;
}


/*
This function checks if the target altitude is reached.
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 10 mm.
*/
bool ARDroneControllerNode::checkIfAltitudeReached(int inputNumberOfMillimetersToTarget)
{
return fabs(altitude - targetAltitude) < inputNumberOfMillimetersToTarget;
}

/*
This function checks if a command has been completed and should be removed from the queue.
@return: true if the command has been completed and false otherwise
*/
bool ARDroneControllerNode::checkIfCommandCompleted(const command &inputCommand)
{

switch(inputCommand.type)
{
case INVALID:
return true; //Invalid commands have always been completed
break;

case SET_CLEAR_COMMAND_QUEUE: //Clear the command queue
case SET_EMERGENCY_LANDING_COMMAND:
case SET_EMERGENCY_STOP_COMMAND:
return false;
break;

case SET_TAKEOFF_COMMAND:
return checkIfHoveringStateAchieved();
break;

case SET_LANDING_COMMAND:
return checkIfLandedStateAchieved();
break;

case SET_CONTROL_SHUTDOWN_COMMAND:
case SET_TARGET_ALTITUDE_COMMAND:
case SET_HORIZONTAL_HEADING_COMMAND:
case SET_ANGULAR_VELOCITY_COMMAND:
case SET_FLIGHT_ANIMATION_COMMAND:
case SET_LED_ANIMATION_COMMAND:
return false;
break;

case SET_WAIT_COMMAND:
return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0;
break;

case SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND:
//Return true if the timer has expired or a tag has been spotted
return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0 || checkIfTagSpotted();
break;

case SET_WAIT_UNTIL_ALTITUDE_REACHED:
//Return true if the timer has expired or the altitude has been reached
if(inputCommand.integers.size() < 1)
{
return true; //Invalid command, so completed
}

return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0 || checkIfAltitudeReached(inputCommand.integers[0]);
break;
}

return true;
}

/*
This function removes the top command from the queue and does any state adjustment associated with the command being completed (such as marking the AR drone as having landed).
*/
void ARDroneControllerNode::removeCompletedCommand()
{
//Get top command
command commandBuffer;
if(copyNextCommand(commandBuffer) != true)
{
return; //No command to remove
}

//Make sure command is removed when the function returns
SOMScopeGuard commandScopeGuard([&](){popNextCommand();});

switch(commandBuffer.type)
{
case INVALID:
case SET_CLEAR_COMMAND_QUEUE: //Clear the command queue
case SET_EMERGENCY_LANDING_COMMAND:
case SET_EMERGENCY_STOP_COMMAND:
return;
break;

case SET_TAKEOFF_COMMAND:
takingOff = false;
return;
break;

case SET_LANDING_COMMAND:
onTheGroundWithMotorsOff = true;
landing = false;
return;
break;

case SET_CONTROL_SHUTDOWN_COMMAND:
case SET_TARGET_ALTITUDE_COMMAND:
case SET_HORIZONTAL_HEADING_COMMAND:
case SET_ANGULAR_VELOCITY_COMMAND:
case SET_FLIGHT_ANIMATION_COMMAND:
case SET_LED_ANIMATION_COMMAND:
return;
break;

case SET_WAIT_COMMAND:
case SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND:
case SET_WAIT_UNTIL_ALTITUDE_REACHED:
currentlyWaiting = false;
return;
break;
}

}

/*
This function takes the appropriate actions for control given the current commands in the queue and data state.  It calls lower level functions to send out the appropriate commands.
*/
void ARDroneControllerNode::processCurrentCommandsForUpdateCycle()
{
while(true)
{
command commandBuffer;
if(copyNextCommand(commandBuffer) != true)
{
//No command is active, initiate emergency landing
commandBuffer.setEmergencyLandingCommand();
SOM_TRY
addCommand(commandBuffer);
adjustBehavior(commandBuffer);
return; //Set emergency landing and then exit command loop
SOM_CATCH("Error setting emergency landing")
}

if(adjustBehavior(commandBuffer) == true)
{
removeCompletedCommand();
}
else
{
break; //Command will take time to complete, so wait for next navdata update to check
}

}

//Handle low level behaviors using the settings from the adjusted behavior
SOM_TRY
handleLowLevelBehavior();
SOM_CATCH("Error, problem implementing low level behavior\n")
}


/*
This function adjusts and enables/disables low level behavior depending on the given command.
@param inputCommand: The command to execute.
@return: true if the command has been completed and false otherwise (allowing multiple commands to be executed in a single update cycle if they are instant).

@exceptions: This function can throw exceptions
*/
bool ARDroneControllerNode::adjustBehavior(const command &inputCommand)
{
if(checkIfCommandCompleted(inputCommand))
{
return true; //Command completed already, so do nothing
}

switch(inputCommand.type)
{
case INVALID:
break;

case SET_CLEAR_COMMAND_QUEUE: //Clear the command queue
clearCommandQueue();
return true; //Instant commands return true
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
return true; //Instant commands return true
break;

case SET_EMERGENCY_STOP_COMMAND:
SOM_TRY
activateEmergencyStop();
SOM_CATCH("Error activating emergency stop\n")
emergencyStopTriggered = true;
clearCommandQueue();
return true; //Instant commands return true
break;

case SET_TAKEOFF_COMMAND:

if(takingOff != true)
{
SOM_TRY
activateTakeoffSequence();
onTheGroundWithMotorsOff = false;
SOM_CATCH("Error activating takeoff sequence")
takingOff = true;
}
break;

case SET_LANDING_COMMAND:
if(landing != true)
{
SOM_TRY
activateLandingSequence();
SOM_CATCH("Error activating landing sequence")
landing = true;
}
break;

case SET_CONTROL_SHUTDOWN_COMMAND:
shutdownControlEngine = true;
return true; //Instant commands return true
break;

case SET_TARGET_ALTITUDE_COMMAND:
if(inputCommand.doubles.size() > 0)
{
targetAltitude = inputCommand.doubles[0];
targetAltitudeITerm = 0.0;
maintainAltitude = true; //Enable trying to meet a set altitude
}
else
{
throw SOMException("Error, set target altitude command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;

case SET_HORIZONTAL_HEADING_COMMAND:
if(inputCommand.doubles.size() > 1)
{
xHeading = inputCommand.doubles[0];
yHeading = inputCommand.doubles[1];
}
else
{
throw SOMException("Error, set set horizontal heading command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;

case SET_ANGULAR_VELOCITY_COMMAND:
if(inputCommand.doubles.size() > 0)
{
currentAngularVelocitySetting = inputCommand.doubles[0];
}
else
{
throw SOMException("Error, set angular velocity command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;


case SET_FLIGHT_ANIMATION_COMMAND:
if(inputCommand.flightAnimations.size() > 0)
{
SOM_TRY
activateFlightAnimation(inputCommand.flightAnimations[0]);
SOM_CATCH("Error activating flight animation\n")
}
else
{
throw SOMException("Error, set flight animation command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;

case SET_LED_ANIMATION_COMMAND:
if(inputCommand.ledAnimations.size() > 0 && inputCommand.doubles.size() > 0 && inputCommand.integers.size() > 0)
{
SOM_TRY
activateLEDAnimationSequence(inputCommand.ledAnimations[0], inputCommand.doubles[0], inputCommand.integers[0]);
SOM_CATCH("Error activating LED animation\n")
}
else
{
throw SOMException("Error, set led animation command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;

case SET_WAIT_COMMAND:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() > 0)
{
SOM_TRY
waitFinishTime = std::chrono::high_resolution_clock::now()  - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
currentlyWaiting = true;
SOM_CATCH("Error waiting for seconds\n")
}
else
{
throw SOMException("Error, waiting for seconds command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
}
break;

case SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() > 0)
{
SOM_TRY
waitFinishTime = std::chrono::high_resolution_clock::now()  - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
currentlyWaiting = true;
SOM_CATCH("Error waiting for seconds or until tag spotted\n")
}
else
{
throw SOMException("Error, waiting for seconds or until tag spotted command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
}
break;

case SET_WAIT_UNTIL_ALTITUDE_REACHED:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() > 0 && inputCommand.integers.size() > 0)
{
SOM_TRY
waitFinishTime = std::chrono::high_resolution_clock::now()  - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
currentlyWaiting = true;
SOM_CATCH("Error waiting till altitude reached\n")
}
else
{
throw SOMException("Error, waiting to reach altitude command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
break;
}
}

return checkIfCommandCompleted(inputCommand);
}


/*
This function takes care of low level behavior that depends on the specific state variables (such as reaching the desired altitude and orientation).

@exceptions: This function can throw exceptions
*/
void ARDroneControllerNode::handleLowLevelBehavior()
{

if(controlEngineIsDisabled || onTheGroundWithMotorsOff || emergencyStopTriggered || shutdownControlEngine)
{
printf("No control %d %d %d\n", controlEngineIsDisabled,  onTheGroundWithMotorsOff, emergencyStopTriggered);
return; //Return if we shouldn't be trying to fly
}

double zThrottle = 0.0;
if(maintainAltitude)
{
double pTerm = (targetAltitude - fabs(altitude));
targetAltitudeITerm = targetAltitudeITerm + pTerm;

zThrottle = pTerm/600.0 + targetAltitudeITerm/100000000.0; //PI control for altitude

//printf("Altitude values: target: %.1lf Current: %.1lf Diff: %.1lf throt:  %.1lf I:%.1lf\n", targetAltitude, altitude, targetAltitude -altitude, zThrottle, targetAltitudeITerm); 
}


double xThrottle = xHeading;
double yThrottle = yHeading;

double zRotationThrottle = currentAngularVelocitySetting;
if(matchTagOrientation)
{
//Override angular velocity setting to track tag orientation
zRotationThrottle = trackedTags[0].orientation / 180; //Might need offset and other configuration
}

//Tell the drone how it should move
setVelocityAndRotation(xThrottle, yThrottle, zThrottle, zRotationThrottle);
}



/*
This function repeatedly calls ros::spinOnce until the spinThreadExitFlag in the given object is set (usually by the object destructor.  It is usually run in a seperate thread.
@param inputARDroneControllerNode: The node this function call is associated
*/
void initializeAndRunSpinThread(ARDroneControllerNode *inputARDroneControllerNode)
{
//Set camera to look forward for tags
inputARDroneControllerNode->setCameraFront(true);

while(inputARDroneControllerNode->spinThreadExitFlag != true)
{
ros::spinOnce();
}
}

