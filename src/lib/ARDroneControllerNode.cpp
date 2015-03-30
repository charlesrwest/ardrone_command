#include "ARDroneControllerNode.hpp"





/*
This function takes the AR drone address and starts up the nessisary ROS nodes to be able to communicate with it.  If the string is empty, it uses the default AR drone address (currently only default functionality is implemented).  It also initializes the QR code based state estimation engine used for QR code based commands.
@param inputCameraImageWidth: The width of camera images used in the camera calibration
@param inputCameraImageHeight: The height of the camera images used in the camera calibration
@param inputCameraCalibrationMatrix: This is a 3x3 matrix that describes the camera transform (taking the distortion into account) in opencv format
@param inputDistortionParameters: a 1x5 matrix which has the distortion parameters k1, k2, p1, p2, k3
@param inputARDroneAddress: The address of the AR drone to connect to 

@exceptions: This function can throw exceptions
*/
ARDroneControllerNode::ARDroneControllerNode(int inputCameraImageWidth, int inputCameraImageHeight, const cv::Mat_<double> &inputCameraCalibrationMatrix, const cv::Mat_<double> &inputCameraDistortionParameters, std::string inputARDroneAddress) : QRCodeEngine(inputCameraImageWidth, inputCameraImageHeight, inputCameraCalibrationMatrix, inputCameraDistortionParameters, true), targetXYZCoordinate(3)
{
spinThreadExitFlag = false;
controlEngineIsDisabled = false;
shutdownControlEngine = false;
currentlyWaiting = false;

onTheGroundWithMotorsOff = true;
takingOff = false;
landing = false;
emergencyStopTriggered = false;
maintainAltitude = false;
maintainQRCodeDefinedPosition = false;
maintainQRCodeDefinedOrientation = false;
commandCounter = 0;
lastPublishedCommandCount = -1;
targetAltitude = 1000.0; //The altitude to maintain in mm
xHeading = 0.0; 
yHeading = 0.0; 
currentAngularVelocitySetting = 0.0; //The setting of the current velocity

//Subscribe to the nav-data topic with a buffer size of 1000
navDataSubscriber = nodeHandle.subscribe("ardrone/navdata", 1000, &ARDroneControllerNode::handleNavData, this); 

//Subscribe to the video from the AR drone with a buffer size of 5
videoSubscriber = nodeHandle.subscribe("ardrone/front/image_raw", 5, &ARDroneControllerNode::handleImageUpdate, this); 

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

//Initialize publishers to give ardrone_command state info

//Create publisher to publish any state estimations we get from the QR code subsystem
QRCodeStateInfoPublisher = nodeHandle.advertise<ardrone_command::qr_code_state_info>(QR_CODE_STATE_PUBLISHER_STRING, 1000);

//Create publisher to publish what we are currently doing in the way of altitude control
altitudeControlInfoPublisher = nodeHandle.advertise<ardrone_command::altitude_control_state>(ALTITUDE_CONTROL_PUBLISHER_STRING, 1000);

//Create publisher to publish what we are currently doing in the way of controlling to go to a particular point
QRCodeGoToPointControlInfoPublisher = nodeHandle.advertise<ardrone_command::qr_go_to_point_control_info>(QR_CODE_GO_TO_POINT_CONTROL_PUBLISHER_STRING, 1000);

//Create publisher to publish what we are currently doing in the way of tracking so that we are pointed at the QR code
QRCodeOrientationControlInfoPublisher = nodeHandle.advertise<ardrone_command::qr_orientation_control_info>(QR_CODE_ORIENTATION_CONTROL_PUBLISHER_STRING, 1000);

//Create publisher to publish what command the drone is currently processing
commandProcessingInfoPublisher = nodeHandle.advertise<ardrone_command::command_status_info>(COMMAND_PROCESSING_INFO_PUBLISHER_STRING, 1000);

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

//printf("I've %ld commands\n", commandQueue.size());

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
printf("Queue size reached zero\n");
return;
}

commandCounter++; //Increment completed command count

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
commandCounter++; //Increment completed command count
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

//Get current estimated location from QR code for diagnostics
if(QRCodeIDToStateEstimate.count("BigQRCode") > 0)
{

//printf("Camera position: %lf %lf %lf\n", QRCodeIDToStateEstimate["BigQRCode"]->currentCameraPosition[0], QRCodeIDToStateEstimate["BigQRCode"]->currentCameraPosition[1], QRCodeIDToStateEstimate["BigQRCode"]->currentCameraPosition[2]);


/*
printf("QR code position: %lf %lf %lf\n", QRCodeIDToStateEstimate["BigQRCode"]->currentQRCodePosition[0], QRCodeIDToStateEstimate["BigQRCode"]->currentQRCodePosition[1], QRCodeIDToStateEstimate["BigQRCode"]->currentQRCodePosition[2]);
*/

/*
for(int row = 0; row < 4; row++)
{
for(int col = 0; col < 4; col ++)
{
printf("%lf ", QRCodeIDToStateEstimate["BigQRCode"]->cameraPose.at<double>(row, col));
}
printf("\n");
}
printf("\n");
*/
}


 
}

/*
This function is used as a callback to handle images from the AR drone.
@param inputImageMessage: The image message to handle 
*/
void ARDroneControllerNode::handleImageUpdate(const sensor_msgs::ImageConstPtr& inputImageMessage)
{
//Update QR code pose information based on new image
cv_bridge::CvImagePtr openCVBridgeImagePointer;

//printf("Test: %d %d %d %s\n", (int) inputImageMessage->width, (int) inputImageMessage->height, (int) inputImageMessage->step, inputImageMessage->encoding.c_str());

try
{
SOM_TRY
openCVBridgeImagePointer = cv_bridge::toCvCopy(inputImageMessage, sensor_msgs::image_encodings::MONO8);
SOM_CATCH("Error converting from ROS image format to Opencv format")
}
catch(std::exception &inputException)
{
printf("!!!!!!!!!%s\n", inputException.what());
}


std::vector<cv::Mat> cameraPoses;
std::vector<std::string> QRCodeIdentifiers;
std::vector<double> QRCodeDimensions;

//printf("Image size: %d %d\n", openCVBridgeImagePointer->image.cols,openCVBridgeImagePointer->image.rows);


SOM_TRY
QRCodeEngine.estimateOneOrMoreStatesFromGrayscaleFrame(openCVBridgeImagePointer->image, cameraPoses, QRCodeIdentifiers, QRCodeDimensions);
SOM_CATCH("Error getting poses from opencv frame\n")

//Update associated QR code state estimates
for(int i=0; i<QRCodeIdentifiers.size(); i++)
{
if(QRCodeIDToStateEstimate.count(QRCodeIdentifiers[i]) > 0)
{ //This QR code has been seen before, so update the estimator
SOM_TRY
QRCodeIDToStateEstimate[QRCodeIdentifiers[i]]->updatePose(cameraPoses[i], QRCodeDimensions[i]);
SOM_CATCH("Error updating QR code state estimate info\n");
}
else
{ //This is a new QR code, so make a new estimator for it
SOM_TRY
QRCodeIDToStateEstimate[QRCodeIdentifiers[i]].reset(new QRCodeBasedPoseInformation(QRCodeIdentifiers[i], cameraPoses[i], QRCodeDimensions[i]));
SOM_CATCH("Error inserting QRCodeBasedPoseInformation into map\n")
}
}

//Post new pose information so using applications know what is going on
auto currentTime = ros::Time::now();
for(int i=0; i<QRCodeIdentifiers.size(); i++) //Send each pose update
{
ardrone_command::qr_code_state_info message;
message.time_stamp = currentTime;
message.qr_code_identifier = QRCodeIdentifiers[i];
message.qr_code_size = QRCodeDimensions[i];

for(int aa = 0; aa<4; aa++)
{
for(int a = 0; a < 4; a++)
{
message.transform[aa*4+a] = cameraPoses[i].at<double>(aa,a);
}
}

QRCodeStateInfoPublisher.publish(message);
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
This function checks to see if the last state estimate associated with a QR code identifier is either doesn't exist or is older than the given time.
@param inputQRCodeIdentifier: The identifier of the QR code that is defining the state estimate
@param inputSecondsBeforeStale: The number of seconds that can pass before an entry is considered stale
@return: true if the entry is stale and false otherwise
*/
bool ARDroneControllerNode::checkIfQRCodeStateEstimateIsStale(const std::string &inputQRCodeIdentifier, double inputSecondsBeforeStale)
{
if(QRCodeIDToStateEstimate.count(inputQRCodeIdentifier) == 0)
{ //We've never seen the QR code that should be defining our coordinate system
return true;
}

//printf("%ld\n", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - QRCodeIDToStateEstimate[inputQRCodeIdentifier]->cameraPoseUpdateTime).count());

return (std::chrono::high_resolution_clock::now() - QRCodeIDToStateEstimate[inputQRCodeIdentifier]->cameraPoseUpdateTime) > std::chrono::duration<double>(inputSecondsBeforeStale); 
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
case SET_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT:
case SET_CANCEL_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT:
case SET_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE:
case SET_CANCEL_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE:
return false;
break;

case SET_WAIT_COMMAND:
if(!currentlyWaiting)
{
return false; //Haven't set time to wait for yet
}

return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0;
break;

case SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND:
if(!currentlyWaiting)
{
return false; //Haven't set time to wait for yet
}

//Return true if the timer has expired or a tag has been spotted
return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0 || checkIfTagSpotted();
break;

case SET_WAIT_UNTIL_SPECIFIC_QR_CODE_IS_SPOTTED_COMMAND:
//Check if QR code has been spotted in the last quarter second
if(inputCommand.strings.size() == 0 || inputCommand.doubles.size() == 0)
{
return true; //Invalid command, so completed
}

if(!currentlyWaiting)
{
return false; //Haven't set time to wait for yet
}

if(QRCodeIDToStateEstimate.count(inputCommand.strings[0]) == 0)
{
return false; //Tag hasn't ever been seen
}
//Return true if the tag has been seen in the last .25 seconds or the time has run out
return (QRCodeIDToStateEstimate[inputCommand.strings[0]]->cameraPoseUpdateTime - std::chrono::high_resolution_clock::now()) < std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(.25)) || std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0;
break;

case SET_WAIT_UNTIL_POSITION_AT_SPECIFIC_QR_CODE_POINT_REACHED:
{
//Calculate distance from point
if(inputCommand.doubles.size() < 3)
{
return true;  //Invalid command, so completed
}

if(!currentlyWaiting)
{
return false; //Haven't set time to wait for yet
}

//See if we have ever seen the current QR code target
if(QRCodeIDToStateEstimate.count(targetXYZCoordinateQRCodeIdentifier) == 0)
{
return false;
}

//Calculate distance to point
std::vector<double> weightedAverageBuffer = QRCodeIDToStateEstimate[targetXYZCoordinateQRCodeIdentifier]->weightedAverageCameraPosition;

double distance = 0;
for(int i=0; i<targetXYZCoordinate.size(); i++)
{
distance = distance +  pow(targetXYZCoordinate[i] - weightedAverageBuffer[i], 2.0);
}
distance = sqrt(distance);

//Calculate current speed
double speed = sqrt(pow(velocityX, 2.0) + pow(velocityY, 2.0) + pow(velocityZ, 2.0));

//Return true if the wait has finished or the distance is less than the command threshold
return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waitFinishTime).count() >= 0 || (distance < inputCommand.doubles[1] && speed < inputCommand.doubles[2]);
}
break;

case SET_WAIT_UNTIL_ALTITUDE_REACHED:
//Return true if the timer has expired or the altitude has been reached
if(inputCommand.integers.size() < 1)
{
return true; //Invalid command, so completed
}

if(!currentlyWaiting)
{
return false; //Haven't set time to wait for yet
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
printf("Removed completed command\n");
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
case SET_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT:
case SET_CANCEL_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT:
case SET_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE:
case SET_CANCEL_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE:
return;
break;

case SET_WAIT_COMMAND:
case SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND:
case SET_WAIT_UNTIL_SPECIFIC_QR_CODE_IS_SPOTTED_COMMAND:
case SET_WAIT_UNTIL_POSITION_AT_SPECIFIC_QR_CODE_POINT_REACHED:
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
bool gotACommand = copyNextCommand(commandBuffer);

if(gotACommand != true && onTheGroundWithMotorsOff)
{
//No command is active, but we are on the ground, so just wait
commandBuffer.setWaitCommand(.1);
SOM_TRY
addCommand(commandBuffer);
adjustBehavior(commandBuffer);
return; 
SOM_CATCH("Error adding wait when there are no commands")
}

if(gotACommand != true && !onTheGroundWithMotorsOff)
{
//No command is active, initiate emergency landing if in the air
commandBuffer.setEmergencyLandingCommand();
SOM_TRY
addCommand(commandBuffer);
adjustBehavior(commandBuffer);
return; //Set emergency landing and then exit command loop
SOM_CATCH("Error setting emergency landing")
}

if(commandCounter != lastPublishedCommandCount)
{
//We have a new command, so publish that we are working on it
lastPublishedCommandCount = commandCounter;
ardrone_command::command_status_info message;
message.time_stamp = ros::Time::now();
message.commandNumber = commandCounter;
message.command = commandBuffer.serialize();
commandProcessingInfoPublisher.publish(message);
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

case SET_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT:
if(inputCommand.doubles.size() == 3 && inputCommand.strings.size() > 0)
{
maintainQRCodeDefinedPosition = true;
targetXYZCoordinate = inputCommand.doubles;
targetXYZCoordinateQRCodeIdentifier = inputCommand.strings[0];
QRTargetXITerm = 0.0;
QRTargetYITerm = 0.0;
}
else
{
throw SOMException("Error, set QR Code position command invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;

case SET_CANCEL_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT:
maintainQRCodeDefinedPosition = false;
return true; //Instant commands return true
break;

case SET_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE:
if(inputCommand.strings.size() > 0)
{
maintainQRCodeDefinedOrientation = true;
targetOrientationQRCodeIdentifier = inputCommand.strings[0];
}
else
{
throw SOMException("Error, set QR Code orientation target command invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
return true; //Instant commands return true
break;

case SET_CANCEL_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE:
maintainQRCodeDefinedOrientation = false;
return true; //Instant commands return true
break;

case SET_WAIT_COMMAND:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() > 0)
{
SOM_TRY
waitFinishTime = std::chrono::high_resolution_clock::now()  + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
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
waitFinishTime = std::chrono::high_resolution_clock::now()  + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
currentlyWaiting = true;
SOM_CATCH("Error waiting for seconds or until tag spotted\n")
}
else
{
throw SOMException("Error, waiting for seconds or until tag spotted command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
}
break;

case SET_WAIT_UNTIL_SPECIFIC_QR_CODE_IS_SPOTTED_COMMAND:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() > 0 && inputCommand.strings.size() > 0)
{
SOM_TRY
QRCodeToSpotIdentifier = inputCommand.strings[0];
waitFinishTime = std::chrono::high_resolution_clock::now()  + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
currentlyWaiting = true;
SOM_CATCH("Error waiting for seconds or until tag spotted\n")
}
else
{
throw SOMException("Error, waiting for seconds or until QR code spotted command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
}
break;

case SET_WAIT_UNTIL_POSITION_AT_SPECIFIC_QR_CODE_POINT_REACHED:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() == 3)
{
SOM_TRY
waitFinishTime = std::chrono::high_resolution_clock::now()  + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
currentlyWaiting = true;
printf("Got here!\n");
SOM_CATCH("Error waiting for seconds or until tag spotted\n")
}
else
{
throw SOMException("Error, waiting for seconds or until QR code define position is reached command is invalid\n", INCORRECT_SERVER_RESPONSE, __FILE__, __LINE__);
}
}
break;

case SET_WAIT_UNTIL_ALTITUDE_REACHED:
if(!currentlyWaiting)
{
if(inputCommand.doubles.size() > 0 && inputCommand.integers.size() > 0)
{
SOM_TRY
waitFinishTime = std::chrono::high_resolution_clock::now()  + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(inputCommand.doubles[0]));
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
//if((controlEngineIsDisabled || onTheGroundWithMotorsOff || emergencyStopTriggered || shutdownControlEngine) && !maintainQRCodeDefinedPosition)
{
//printf("No control %d %d %d\n", controlEngineIsDisabled,  onTheGroundWithMotorsOff, emergencyStopTriggered);
return; //Return if we shouldn't be trying to fly
}

double zThrottle = 0.0;
if(maintainQRCodeDefinedPosition)
{
//Remove stale I term
targetAltitudeITerm = 0.0;
}

if(maintainAltitude && !maintainQRCodeDefinedPosition)
{
double pTerm = (targetAltitude - fabs(altitude));
targetAltitudeITerm = targetAltitudeITerm + pTerm;

zThrottle = pTerm/600.0 + targetAltitudeITerm/100000000.0; //PI control for altitude

//Send message to log changes from altitude control
if(!maintainQRCodeDefinedPosition)
{
ardrone_command::altitude_control_state message;

message.time_stamp = ros::Time::now();
message.target_altitude = targetAltitude;
message.current_p_term = pTerm;
message.current_i_term = targetAltitudeITerm;

altitudeControlInfoPublisher.publish(message);
}

//printf("Altitude values: target: %.1lf Current: %.1lf Diff: %.1lf throt:  %.1lf I:%.1lf\n", targetAltitude, altitude, targetAltitude -altitude, zThrottle, targetAltitudeITerm); 
}



if(maintainQRCodeDefinedPosition)
{ //Check to see if we have to engage emergency landing due to losing the code
if(checkIfQRCodeStateEstimateIsStale(targetXYZCoordinateQRCodeIdentifier, SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING))
{ //We've never seen the QR code that should be defining our coordinate system or it has been too long since we have seen it
SOM_TRY
printf("Tracking lost\n");
maintainQRCodeDefinedPosition = false;
activateLandingSequence();
clearCommandQueue();
SOM_CATCH("Error triggering emergency landing and clearing command queue")
}
}


if(maintainQRCodeDefinedOrientation)
{ //Check to see if we have to engage emergency landing due to losing the code
if(checkIfQRCodeStateEstimateIsStale(targetOrientationQRCodeIdentifier, SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING))
{ //We've never seen the QR code that should be defining our coordinate system or it has been too long since we have seen it
SOM_TRY
printf("Tracking lost\n");
maintainQRCodeDefinedOrientation = false;
activateLandingSequence();
clearCommandQueue();
SOM_CATCH("Error triggering emergency landing and clearing command queue")
}
}


double xThrottle = xHeading;
double yThrottle = yHeading;



if(maintainQRCodeDefinedPosition)
{
ardrone_command::qr_go_to_point_control_info message;

//Convert target point to the drone's coordinate system
std::vector<double> localTargetPoint = QRCodeIDToStateEstimate[targetXYZCoordinateQRCodeIdentifier]->convertPointInQRCodeSpaceToCameraSpace(targetXYZCoordinate);

std::vector<double> cameraPosition = QRCodeIDToStateEstimate[targetXYZCoordinateQRCodeIdentifier]->currentCameraPosition;
//printf("Current camera position: %lf %lf %lf\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);

//printf("Target point: %lf %lf %lf                Mag:%lf\n", localTargetPoint[2], localTargetPoint[0], localTargetPoint[1], sqrt(pow(localTargetPoint[0], 2) + pow(localTargetPoint[1], 2) + pow(localTargetPoint[2], 2) ));

QRTargetXITerm = QRTargetXITerm + localTargetPoint[2];
QRTargetYITerm = QRTargetYITerm + localTargetPoint[0] ;

double distance = sqrt(pow(localTargetPoint[0], 2) + pow(localTargetPoint[1], 2) + pow(localTargetPoint[2], 2) );

//Use one PID set if close, another if far
/*
if(distance < 1)
{ //Near
*/
message.mode = 0; //Mark as near
printf("Near\n");
//Camera xyz maps to ardrone (-y)xz
xThrottle = -.0003*velocityX+ .15*localTargetPoint[2]  + .00001*QRTargetXITerm ; //Simple PI control for now
yThrottle = .0003*velocityY+ .15*localTargetPoint[0] + .00001*QRTargetYITerm ; 
zThrottle = -.1*localTargetPoint[1]; 
/*
}
else
{//Far
printf("Far\n");
message.mode = 1; //Mark far
//Camera xyz maps to ardrone (-y)xz
xThrottle = -.0004*velocityX+ .1*localTargetPoint[2]  + .00001*QRTargetXITerm ; //Simple PI control for now
yThrottle = .0004*velocityY+ .1*localTargetPoint[0] + .00001*QRTargetYITerm ; 
zThrottle = -.05*localTargetPoint[1]; 
}
*/

//Override control to make drone hover if experiencing high latency (but not high enough to make it land)
if(checkIfQRCodeStateEstimateIsStale(targetXYZCoordinateQRCodeIdentifier, HIGH_LATENCY_WATER_MARK))
{
xThrottle = 0.0;
yThrottle = 0.0;
zThrottle = 0.0;
printf("Experiencing high latency\n");
}


//Get ready to send message detailing control stuff
for(int i=0; i<localTargetPoint.size(); i++)
{
message.target_point_camera_xyz[i] =  localTargetPoint[i];
}
message.target_point_local_xyz[0] = localTargetPoint[2];
message.target_point_local_xyz[1] = localTargetPoint[0];
message.target_point_local_xyz[2] = localTargetPoint[1];
message.estimated_distance_to_target = distance;
message.qr_xyz_throttle[0] = xThrottle;
message.qr_xyz_throttle[1] = yThrottle;
message.qr_xyz_throttle[2] = zThrottle;
message.qr_x_axis_I_term = QRTargetXITerm;
message.qr_y_axis_I_term = QRTargetYITerm;
message.time_stamp = ros::Time::now();

//Send status message
QRCodeGoToPointControlInfoPublisher.publish(message);


//printf("Throttle: %lf %lf %lf  Point: %lf %lf %lf\n", xThrottle, yThrottle, zThrottle, localTargetPoint[2], localTargetPoint[0], localTargetPoint[1]);
}

double zRotationThrottle = currentAngularVelocitySetting;
if(maintainQRCodeDefinedOrientation)
{

std::vector<double> QRCodePoint = QRCodeIDToStateEstimate[targetOrientationQRCodeIdentifier]->currentQRCodePosition;

//printf("Current QR code position: %lf\n", QRCodePoint[0]);

//Override angular velocity setting to track tag orientation
zRotationThrottle = -2.0*QRCodePoint[0]/fabs(QRCodePoint[2]); //Simple bang/bang control for now

//Make status message to send
ardrone_command::qr_orientation_control_info message;
message.time_stamp = ros::Time::now();
message.z_rotation_throttle = zRotationThrottle;

//Send message
QRCodeOrientationControlInfoPublisher.publish(message);
}

//Tell the drone how it should move TODO: Change back
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

