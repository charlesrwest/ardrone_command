#ifndef COMMANDHPP
#define COMMANDHPP

#include<vector>
#include "ARDroneEnums.hpp"
#include<cstdint>

#define MAX_PRIORITY 1000

enum commandType
{
INVALID,
SET_CLEAR_COMMAND_QUEUE,
SET_EMERGENCY_LANDING_COMMAND,
SET_EMERGENCY_STOP_COMMAND,
SET_TAKEOFF_COMMAND,
SET_LANDING_COMMAND,
SET_CONTROL_SHUTDOWN_COMMAND,
SET_TARGET_ALTITUDE_COMMAND,
SET_HORIZONTAL_HEADING_COMMAND,
SET_ANGULAR_VELOCITY_COMMAND,
SET_HOME_IN_ON_TAG_COMMAND,
SET_CANCEL_HOME_IN_ON_TAG_COMMAND,
SET_MATCH_ORIENTATION_TO_TAG_COMMAND,
SET_CANCEL_MATCH_ORIENTATION_TO_TAG_COMMAND,
SET_FLIGHT_ANIMATION_COMMAND,
SET_LED_ANIMATION_COMMAND,
SET_WAIT_COMMAND,
SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND,
SET_WAIT_UNTIL_ALTITUDE_REACHED
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
This function clears the command and then tells the drone to seek in on the first tag in its vision.  This command's effect persists until canceled, but will not cause any changes in horizontal velocity if no tag is in view. 
*/
void setHomeInOnTagCommand();

/*
This function clears the command and then cancels the home in on tag behavior. 
*/
void setCancelHomeInOnTagCommand();

/*
This function clears the command and then tells the drone to try to match the orientation of the first tag in view.  This command's effect persists until canceled, but will not cause any changes in angular velocity if no tag is in view. 
*/
void setMatchOrientationToTagCommand();

/*
This function clears the command and then tells the drone to cancel the match orientation to tag command. 
*/
void setCancelMatchOrientationToTagCommand();

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



#endif
