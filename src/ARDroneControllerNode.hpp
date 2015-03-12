#ifndef ARDRONECONTROLLERNODEHPP
#define ARDRONECONTROLLERNODEHPP

#include<mutex>
#include<thread>
#include<chrono>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "tagTrackingInfo.hpp"
#include "ARDroneEnums.hpp"
#include "command.hpp"

#include "SOMException.hpp"
#include "SOMScopeGuard.hpp"

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <ardrone_autonomy/vector31.h>
#include <ardrone_autonomy/vector21.h>
#include <ardrone_autonomy/matrix33.h>

#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>
#include <ardrone_autonomy/FlightAnim.h>
#include <ardrone_autonomy/RecordEnable.h>
#include <ardrone_autonomy/Navdata.h>

#include <queue>
#include <condition_variable>
#include <chrono>

//This defines how long to wait for a QR code sighting when in a mode reliant on QR code state estimation before automatically landing
#define SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING 2

/*
This object subscribes to a set of ROS information streams that are available regarding a AR drone (some of which may be aggregators of other streams) and publishes a command stream to control the drone using information gleamed from those streams.  This object also has commands available to launch the other nodes so that it may subscribe to them.
*/
class ARDroneControllerNode
{
public:
/*
This function takes the AR drone address and starts up the nessisary ROS nodes to be able to communicate with it.  If the string is empty, it uses the default AR drone address (currently only default functionality is implemented).  If a joystick node is being published, the node will also subscribe to that and interpret its movements as commands to send to the drone (if the manual flight flag is set)
@param inputARDroneAddress: The address of the AR drone to connect to 

@exceptions: This function can throw exceptions
*/
ARDroneControllerNode(std::string inputARDroneAddress = "");

/*
This function allows a command to be added to the queue of commands for the drone to execute.  This function is threadsafe, so it can be used from multiple threads but may sometimes block.
@param inputCommand: The command to add

@exceptions: This function can throw exceptions
*/
void addCommand(const command &inputCommand);

/*
This function allows an outside thread to see how many commands there are in the current queue.  This does not count the one (if any) currently executing. This function is threadsafe (and so may sometimes block).
@return: The number of commands in the current queue

@exceptions: This function can throw exceptions
*/
int commandQueueSize();

/*
This function cleans up the object and waits for any threads the object spawned to return.
*/
~ARDroneControllerNode();


bool manualControlEnabled; //Not currently set up

friend void initializeAndRunSpinThread(ARDroneControllerNode *inputARDroneControllerNode);
friend void initializeAndRunControlThread(ARDroneControllerNode *inputARDroneControllerNode);

//Only used as callback

/*
This function is used as a callback to handle legacy nav-data.
@param inputMessage: The legacy nav-data message to handle 
*/
void handleLegacyNavData(const ardrone_autonomy::Navdata::ConstPtr &inputMessage);


private:
/*
This threadsafe function puts the next available command in the buffer and returns false if there was no command to give.  Retrieved commands are removed from the buffer.
@param inputCommandBuffer: The next command in the queue or the same as before if no command was available
@return: True if there was a new command and false otherwise
*/
bool getNextCommand(command &inputCommandBuffer);

/*
This threadsafe function removes all of the elements in the command queue
*/
void clearCommandQueue();

/*
This function is called by the control thread to execute the commands in the command queue.  The function normally does not return unless it receives the control shutdown command.
@exceptions: This function can throw exceptions
*/
void controlDrone();

/*
This function sends a message to tell the drone to takeoff using its built in takeoff sequence
@exceptions: This function can throw exceptions
*/
void activateTakeoffSequence();

/*
This function sends a message to tell the drone to land using its built in landing sequence
@exceptions: This function can throw exceptions
*/
void activateLandingSequence();

/*
This function sends a message to tell the drone to active its emergency stop sequence (cutting power to the engines)
@exceptions: This function can throw exceptions
*/
void activateEmergencyStop();

/*
This function sends a message to set the linear and angular velocity of the drone
@param inputVelocityX: The X velocity of the drone
@param inputVelocityY: The Y velocity of the drone
@param inputVelocityZ: The Z velocity of the drone
@param inputRotationZ: The Z rotation rate of the drone
@exceptions: This function can throw exceptions
*/
void setVelocityAndRotation(double inputVelocityX, double inputVelocityY, double inputVelocityZ, double inputRotationZ);

/*
This function sets the linear and angular velocity of the drone to zero and enables the auto-hover mode to try to maintain its position
@exceptions: This function can throw exceptions
*/
void enableAutoHover();

/*
This function either sets the active camera to the front facing one or the bottom one
@param inputSetCameraFront: True if the front camera should be set active, false if the bottom camera should be

@exceptions: This function can throw exceptions
*/
void setCameraFront(bool inputSetCameraFront);

/*
This function will trigger an LED animation sequence on the quadcopter
@param inputAnimationType: The type of the animation
@param inputFrequency: The frequency of the animation (if blinking), in hertz
@param inputDuration: The duration of the animation in seconds

@exceptions: This function can throw exceptions
*/
void activateLEDAnimationSequence(LEDAnimationType inputAnimationType, double inputFrequency, int inputDuration);

/*
This function will trigger a flight animation
@param inputFlightAnimationType: What type of flight animation to perform

@exceptions: This function can throw exceptions
*/
void activateFlightAnimation(flightAnimationType inputFlightAnimationType);

/*
This function causes the drone to recalibrate its rotations using the assumption that it is on a flat service.  Don't use it when it is not on a flat surface.
@exceptions: This function can throw exceptions
*/
void calibrateFlatTrim();

/*
This function causes the drone to start recording to its USB stick (if it has one) or stop recording.
@param inputStartRecording: Set to true if the drone should start recording and false if it should stop
@exceptions: This function can throw exceptions
*/
void enableUSBRecording(bool inputStartRecording);

/*
This function waits on the given locked mutex until the signal for the next navdata update occurs (though there can be fake wakeups)
@param inputLockedUniqueLock: A locked unique lock that has the navdata mutex.  It is returned in a locked state again

@exceptions: This function can throw exceptions
*/
void waitUntilNextNavDataUpdate(std::unique_lock<std::mutex> &inputLockedUniqueLock);

/*
This function blocks until the ARDrone has achieved the hovering state.  This is normally used after the takeoff signal has been sent to wait until the drone finishes taking off.  It doesn't update low level behaviors.

@exceptions: This function can throw exceptions
*/
void waitUntilHoveringStateAchieved();

/*
This function blocks until the ARDrone has achieved the landed state.  This is normally used after the land signal has been sent to wait until the drone finishes setting down.  It updates low level behaviors.

@exceptions: This function can throw exceptions
*/
void waitUntilLandedStateAchieved();

/*
This function blocks until the given amount of time has passed, while maintaining low level behaviors (tag tracking and altitude control).
@param inputSecondsToWait: The number of seconds to wait

@exceptions: This function can throw exceptions
*/
void waitSeconds(double inputSecondsToWait);

/*
This function blocks until the given amount of time has passed or a tag has been spotted, while maintaining low level behaviors (tag tracking and altitude control).
@param inputSecondsToWait: The number of seconds to wait

@exceptions: This function can throw exceptions
*/
void waitSecondsOrUntilTagSpotted(double inputSecondsToWait);

/*
This function blocks until the given amount of time has passed or the target altitude is reached, while maintaining low level behaviors (tag tracking and altitude control).
@param inputNumberOfSeconds: The number of seconds to wait
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 1 mm.

@exceptions: This function can throw exceptions
*/
void waitSecondsOrUntilAltitudeReached(double inputNumberOfSeconds, int inputNumberOfMillimetersToTarget);

/*
This function takes care of low level behavior that depends on the specific state variables (such as reaching the desired altitude, orientation, and following tags).  It assumes that the navdata mutex is locked by the function that called it, so that it can access navdata freely.  This function is typically called in waiting functions after each navdata update. 
@param inputLockedUniqueLock: A locked unique lock that has the navdata mutex.  It is returned in a locked state again

@exceptions: This function can throw exceptions
*/
void handleLowLevelBehavior(std::unique_lock<std::mutex> &inputLockedUniqueLock);

bool controlEngineIsDisabled;
bool onTheGroundWithMotorsOff;
bool takingOff;
bool landing;
bool emergencyStopTriggered;
bool maintainAltitude;
bool homeInOnTag;
bool matchTagOrientation;
double targetAltitude; //The altitude to maintain in mm
double targetAltitudeITerm;
double xHeading; //The current velocity settings of the drone
double yHeading; 
double xCoordinateITerm;
double yCoordinateITerm;
double currentAngularVelocitySetting; //The setting of the current velocity
bool highPriorityCommandWaiting; //True if a high priority command has been added to the queue

//Mutex protected, only public access through threadsafe addCommand function and threadsafe commandQueueSize function
std::mutex commandMutex;
std::queue<command> commandQueue;

//Mutex protected, changed as data becomes available
std::condition_variable newNavDataAvailable;
std::mutex navDataMutex;

enum droneCurrentState state;  //The current state of the drone
double batteryPercent;  //Percentage of the drone's battery remaining
double rotationX; //Left/Right tilt in degrees
double rotationY; //Forward/backward tilt in degrees
double rotationZ; //Turn rotation estimate
double magneticX; //Magnetic reading in X
double magneticY; //Magnetic reading in Y
double magneticZ; //Magnetic reading in Z
double pressure;  //Pressure sensed by barometer (Pa)
double temperature; //Temperature reading
double windSpeed;  //Estimated wind speed
double windAngle; //Estimated wind angle
double windCompensationAngle;  //?
double altitude;   //Estimated altitude (mm)
double motorPWM[4];//Current PWM values for the motors
double velocityX;  //Current estimated X velocity (mm/s)
double velocityY;  //Current estimated Y velocity (mm/s)
double velocityZ;  //Current estimated Z velocity (mm/s)
double accelerationX;  //Current estimated X acceleration
double accelerationY;  //Current estimated Y acceleration
double accelerationZ;  //Current estimated Z acceleration
std::vector<tagTrackingInfo> trackedTags; //Information about any oriented roundel tags in the field of view




ros::NodeHandle nodeHandle;

bool spinThreadExitFlag; //This flag indicates if thread that is calling spinOnce should exit
std::unique_ptr<std::thread> spinThread;
std::unique_ptr<std::thread> controlThread;


ros::Subscriber legacyNavigationDataSubscriber; 

ros::Publisher takeOffPublisher;
ros::Publisher landingPublisher;
ros::Publisher emergencyStopPublisher;
ros::Publisher directionAndSpeedPublisher;

ros::ServiceClient cameraControlClient;
ros::ServiceClient setLEDAnimationClient;
ros::ServiceClient setFlightAnimationClient;
ros::ServiceClient calibrateFlatTrimClient;
ros::ServiceClient setUSBRecordingClient;
};


/*
This function repeatedly calls ros::spinOnce until the spinThreadExitFlag in the given object is set (usually by the object destructor.  It is usually run in a seperate thread.
@param inputARDroneControllerNode: The node this function call is associated
*/
void initializeAndRunSpinThread(ARDroneControllerNode *inputARDroneControllerNode);

/*
This function calls controlDrone until the controlThreadExitFlag in the given object is set (usually by the object destructor.  It is usually run in a seperate thread.
@param inputARDroneControllerNode: The node this function call is associated
*/
void initializeAndRunControlThread(ARDroneControllerNode *inputARDroneControllerNode);





#endif
