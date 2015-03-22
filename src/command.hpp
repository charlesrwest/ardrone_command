#ifndef COMMANDHPP
#define COMMANDHPP

#include<vector>
#include<string>
#include "ARDroneEnums.hpp"
#include<cstdint>
#include "ardrone_command/serialized_ardrone_command.h"


#define MAX_PRIORITY 1000

enum commandType
{
INVALID = 10,
SET_CLEAR_COMMAND_QUEUE = 20,
SET_EMERGENCY_LANDING_COMMAND = 30,
SET_EMERGENCY_STOP_COMMAND = 40,
SET_TAKEOFF_COMMAND = 50,
SET_LANDING_COMMAND = 60,
SET_CONTROL_SHUTDOWN_COMMAND = 70,
SET_TARGET_ALTITUDE_COMMAND = 80,
SET_HORIZONTAL_HEADING_COMMAND = 90,
SET_ANGULAR_VELOCITY_COMMAND = 100,
SET_FLIGHT_ANIMATION_COMMAND = 110,
SET_LED_ANIMATION_COMMAND = 120,

SET_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT = 130,
SET_CANCEL_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT = 140,
SET_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE = 150,
SET_CANCEL_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE = 160,

SET_WAIT_COMMAND = 170,
SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND = 180,
SET_WAIT_UNTIL_SPECIFIC_QR_CODE_IS_SPOTTED_COMMAND = 190,
SET_WAIT_UNTIL_POSITION_AT_SPECIFIC_QR_CODE_POINT_REACHED = 200,
SET_WAIT_UNTIL_ALTITUDE_REACHED = 210
};

/*
This class is a simple interface to allow the creation and queueing of higher level commands for the ARDrone to execute.  It allows nesting of other commands so that more complex commands can be made from simpler ones (with the resulting instructions being decomposed when they are placed in the queue and the command that stored them ignored).  Different construction functions are used to initialize different types of commands easily.
*/
class command
{
public:
/*
Default constructor.  Does nothing.
*/
command();

/*
This function decomposes the command into a ROS message for sending over the network.  ROS messages don't support containing submessages of the same type as the message, so this requires all subcommands to be decomposed into a list of simple (nonrecursive) commands.
*/
ardrone_command::serialized_ardrone_command serialize();

/*
This function clears the command and then creates a command that will have maximum priority and clear all of the remaining commands in the command queue.
*/
void setClearCommandQueueCommand();

/*
This function clears the command and then creates a command that will have maximum priority, set the drone to land and clear all of the remaining commands in the command queue.
*/
void setEmergencyLandingCommand();

/*
This function clears the command and then creates a command that will have maximum priority, set the drone turn off all motors and clear all of the remaining commands in the command queue.
*/
void setEmergencyStopCommand();

/*
This function clears the command and sets it with the parameters appropriate to be a takeoff command.
*/
void setTakeoffCommand();

/*
This function clears the command and sets it with the parameters appropriate to be a landing command.
*/
void setLandingCommand();

/*
This command tells the control software to shutdown when it is encountered, freeing the used resources.
*/
void setControlShutdownCommand();

/*
This function clears the command and makes it so that the drone will move to the appropriate altitude (and not do anything else until it reaches it).
@param inputTargetAltitude: The altitude to go to
*/
void setTargetAltitudeCommand(double inputTargetAltitude);

/*
This function clears the command and makes it so that the drone will move in the specified horizontal heading.
@param inputXSpeed: The speed in the X direction (-1.0, 1.0)
@param inputYSpeed: The speed in the Y direction (-1.0, 1.0)
*/
void setHorizontalHeadingCommand(double inputXSpeed, double inputYSpeed);

/*
This function clears the command and then sets the angular velocity of the drone about the Z axis.
@param inputAngularVelocity: The angular velocity of the drone (-1.0, 1.0)
*/
void setAngularVelocityCommand(double inputAngularVelocity);

/*
This function clears the command and then sets the flight animation to perform.
@param inputFlightAnimation: The type of flight animation to perform
*/
void setFlightAnimationCommand(flightAnimationType inputFlightAnimation);

/*
This function clears the command and then sets the LED animation to perform.
@param inputLEDAnimation: The type of LED animation to perform
@param inputFrequency: The frequency to have the LEDs flash at
@param inputDuration: The number of seconds to have the LEDs flash
*/
void setLEDAnimationCommand(LEDAnimationType inputLEDAnimation, double inputFrequency, int inputDuration);

/*
This function clears the command and then tells the drone to attempt to maintain its position at the given (specific) QR code defined location while it does other things (such as wait).  This is a QR code state estimation based command, which means that landing will automatically engage if the designated QR code has not been seen within SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING.
@param inputQRCodeID: The string identifying the QR code to use for state estimation (will use any if an empty string is passed)
@param inputXCoordinate: The x coordinate in the QR code coordinate system (meters)
@param inputYCoordinate: The y coordinate in the QR code coordinate system (meters)
@param inputZCoordinate: The z coordinate in the QR code coordinate system (meters)
*/
void setMaintainPositionAtSpecificQRCodePoint(const std::string &inputQRCodeID, double inputXCoordinate, double inputYCoordinate, double inputZCoordinate);

/*
This function clears the command and then tells the drone to stop trying to maintain/go to a specific point in the QR code coordinate system.
*/
void setCancelMaintainPositionAtSpecificQRCodePoint();

/*
This function clears the command and then tells the drone to point at a specific QR code in its point that is defining its coordinate system.  This is a QR code state estimation based command, which means that landing will automatically engage if the designated QR code has not been seen within SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING.
@param inputQRCodeID: The string identifying the QR code to use for state estimation (will use any if an empty string is passed)
*/
void setMaintainOrientationTowardSpecificQRCode(const std::string &inputQRCodeID);

/*
This function clears the command and then tells the drone to stop trying to point at a specific QR code in its point that is defining its coordinate system.
*/
void setCancelMaintainOrientationTowardSpecificQRCode();


/*
This function clears the command and then tells the drone to simply maintain its current state for the given number of seconds.
@param inputNumberOfSeconds: The number of seconds to wait
*/
void setWaitCommand(double inputNumberOfSeconds);

/*
This function clears the command and then tells the drone to maintain its current state for the given number of seconds until a tag comes into its field of view.
@param inputNumberOfSeconds: The number of seconds to wait
*/
void setWaitUntilTagIsSpottedCommand(double inputNumberOfSeconds);

/*
This function clears the command and then tells the drone to maintain its current state for the given number of seconds until a specific QR code tag comes into its field of view.
@param inputQRCodeID: The string identifying the QR code to use for state estimation (will use any if an empty string is passed)
@param inputNumberOfSeconds: The number of seconds to wait
*/
void setWaitUntilSpecificQRCodeIsSpottedCommand(const std::string inputQRCodeID, double inputNumberOfSeconds);

/*
This function clears the command and then tells the drone to maintain its current state (seeking the QR code) for the given number of seconds until the weighted average position reaches within a certain distance of the target point.
@param inputNumberOfSeconds: The number of seconds to wait
@param inputTargetDistance: The distance from the point before the goal is considered achieved (in meters)
*/
void setWaitUntilPositionAtSpecificQRCodePointReachedCommand(double inputNumberOfSeconds, double inputTargetDistance = .2);


/*
This function clears the command and then tells the drone to maintain its current state for the given number of seconds until a tag comes into its field of view.
@param inputNumberOfSeconds: The number of seconds to wait
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 1 mm.
*/
void setWaitUntilAltitudeReached(double inputNumberOfSeconds, int inputNumberOfMillimetersToTarget = 10);

/*
This function is used by the priority queue to sort the order of the commands in the queue.  It less the < operator be used to compare the priority of two commands.
@param inputLeftCommand: The command on the left side of the < symbol
@param inputRightCommand: The command on the right side of the < symbol
@return: True if the left command is less than the right command
*/
friend bool operator< (const command &inputLeftCommand, const command &inputRightCommand);

commandType type; //What type of command this is
std::vector<command> subCommands;
std::vector<std::string> strings;
std::vector<double> doubles;
std::vector<int> integers;
std::vector<flightAnimationType> flightAnimations;
std::vector<LEDAnimationType> ledAnimations;
int priority; //How this particular command should be organized in the queue (higher comes out faster)

private:
/*
This function clears the command
*/
void clear();
};



/*
This function is used by the priority queue to sort the order of the commands in the queue.  It less the < operator be used to compare the priority of two commands.
@param inputLeftCommand: The command on the left side of the < symbol
@param inputRightCommand: The command on the right side of the < symbol
@return: True if the left command is less than the right command
*/
bool operator< (const command &inputLeftCommand, const command &inputRightCommand);

/*
This function adds the command and then recursively goes through any subcommands in the command, adding them to the list of commands (depth first search).  The resulting list only contains commands without subcommands.  This is typically used to break down commands for serialization with the ROS message type.
@param inputCommand: The top command to process
@param inputCommandList: The list to start appending to
*/
void unwrapCommand(const command &inputCommand, std::vector<command> &inputCommandList);

/*
This function serializes the command to a serialized_ardrone_command_part, ignoring any subcommands the command may have.
@param inputCommand: The command to serialize
*/
ardrone_command::serialized_ardrone_command_part serializeCommandPart(const command &inputCommand);

/*
This function converts a serialized command into a list of commands/subcommands.
@param inputSerializedCommand: The message to deserialize
@return: The list of commands stored in the message
*/
std::vector<command> deserialize_commands(const ardrone_command::serialized_ardrone_command &inputSerializedCommand);

/*
This function converts a serialized command part into a command
@param inputSerializedCommandPart: The message to deserialize
@return: The command stored in the message
*/
command deserialize_command_part(const ardrone_command::serialized_ardrone_command_part &inputSerializedCommandPart);

#endif
