#include "command.hpp"

/*
Default constructor.  Does nothing.
*/
command::command()
{
type = INVALID;
priority = 0; //Standard priority
}

/*
This function clears the command and then creates a command that will have maximum priority and clear all of the remaining commands in the command queue.
*/
void command::setClearCommandQueueCommand()
{
clear();
priority = MAX_PRIORITY;
type = SET_CLEAR_COMMAND_QUEUE;
}

/*
This function clears the command and then creates a command that will have maximum priority, set the drone to land and clear all of the remaining commands in the command queue.
*/
void command::setEmergencyLandingCommand()
{
clear();
priority = MAX_PRIORITY;
type = SET_EMERGENCY_LANDING_COMMAND;
}

/*
This function clears the command and then creates a command that will have maximum priority, set the drone turn off all motors and clear all of the remaining commands in the command queue.
*/
void command::setEmergencyStopCommand()
{
clear();
priority = MAX_PRIORITY;
type = SET_EMERGENCY_STOP_COMMAND;
}

/*
This function clears the command and sets it with the parameters appropriate to be a takeoff command.
*/
void command::setTakeoffCommand()
{
clear();
type = SET_TAKEOFF_COMMAND;
}

/*
This function clears the command and sets it with the parameters appropriate to be a landing command.
*/
void command::setLandingCommand()
{
clear();
type = SET_LANDING_COMMAND;
}

/*
This command tells the control software to shutdown when it is encountered, freeing the used resources.
*/
void command::setControlShutdownCommand()
{
clear();
type = SET_CONTROL_SHUTDOWN_COMMAND;
}

/*
This function clears the command and makes it so that the drone will move to the appropriate altitude (and not do anything else until it reaches it).
@param inputTargetAltitude: The altitude to go to
*/
void command::setTargetAltitudeCommand(double inputTargetAltitude)
{
clear();
type = SET_TARGET_ALTITUDE_COMMAND;
doubles.push_back(inputTargetAltitude);
}

/*
This function clears the command and makes it so that the drone will move in the specified horizontal heading.
@param inputXSpeed: The speed in the X direction (-1.0, 1.0)
@param inputYSpeed: The speed in the Y direction (-1.0, 1.0)
*/
void command::setHorizontalHeadingCommand(double inputXSpeed, double inputYSpeed)
{
clear();
type = SET_HORIZONTAL_HEADING_COMMAND;
doubles.push_back(inputXSpeed);
doubles.push_back(inputYSpeed);
}

/*
This function clears the command and then sets the angular velocity of the drone about the Z axis.
@param inputAngularVelocity: The angular velocity of the drone (-1.0, 1.0)
*/
void command::setAngularVelocityCommand(double inputAngularVelocity)
{
clear();
type = SET_ANGULAR_VELOCITY_COMMAND;
doubles.push_back(inputAngularVelocity);
}

/*
This function clears the command and then tells the drone to seek in on the first tag in its vision.  This command's effect persists until canceled, but will not cause any changes in horizontal velocity if no tag is in view. 
*/
void command::setHomeInOnTagCommand()
{
clear();
type = SET_HOME_IN_ON_TAG_COMMAND;
}

/*
This function clears the command and then cancels the home in on tag behavior. 
*/
void command::setCancelHomeInOnTagCommand()
{
clear();
type = SET_CANCEL_HOME_IN_ON_TAG_COMMAND;
}

/*
This function clears the command and then tells the drone to try to match the orientation of the first tag in view.  This command's effect persists until canceled, but will not cause any changes in angular velocity if no tag is in view. 
*/
void command::setMatchOrientationToTagCommand()
{
clear();
type = SET_MATCH_ORIENTATION_TO_TAG_COMMAND;
}

/*
This function clears the command and then tells the drone to cancel the match orientation to tag command. 
*/
void command::setCancelMatchOrientationToTagCommand()
{
clear();
type = SET_CANCEL_MATCH_ORIENTATION_TO_TAG_COMMAND;
}

/*
This function clears the command and then sets the flight animation to perform.
@param inputFlightAnimation: The type of flight animation to perform
*/
void command::setFlightAnimationCommand(flightAnimationType inputFlightAnimation)
{
clear();
type = SET_FLIGHT_ANIMATION_COMMAND;
flightAnimations.push_back(inputFlightAnimation);
}

/*
This function clears the command and then sets the LED animation to perform.
@param inputLEDAnimation: The type of LED animation to perform
@param inputFrequency: The frequency to have the LEDs flash at
@param inputDuration: The number of seconds to have the LEDs flash
*/
void command::setLEDAnimationCommand(LEDAnimationType inputLEDAnimation, double inputFrequency, int inputDuration)
{
clear();
type = SET_LED_ANIMATION_COMMAND;
ledAnimations.push_back(inputLEDAnimation);
doubles.push_back(inputFrequency);
integers.push_back(inputDuration);
}

/*
This function clears the command and then tells the drone to simply maintain its current state for the given number of seconds.
@param inputNumberOfSeconds: The number of seconds to wait
*/
void command::setWaitCommand(double inputNumberOfSeconds)
{
clear();
type = SET_WAIT_COMMAND;
doubles.push_back(inputNumberOfSeconds);
}

/*
This function clears the command and then tells the drone to maintain its current state for the given number of seconds until a tag comes into its field of view.
@param inputNumberOfSeconds: The number of seconds to wait
*/
void command::setWaitUntilTagIsSpottedCommand(double inputNumberOfSeconds)
{
clear();
type = SET_WAIT_UNTIL_TAG_IS_SPOTTED_COMMAND;
doubles.push_back(inputNumberOfSeconds);
}

/*
This function clears the command and then tells the drone to maintain its current state for the given number of seconds until a tag comes into its field of view.
@param inputNumberOfSeconds: The number of seconds to wait
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 1 mm.
*/
void command::setWaitUntilAltitudeReached(double inputNumberOfSeconds, int inputNumberOfMillimetersToTarget)
{
clear();
type = SET_WAIT_UNTIL_ALTITUDE_REACHED;
doubles.push_back(inputNumberOfSeconds);
integers.push_back(inputNumberOfMillimetersToTarget);
}

/*
This function clears the command
*/
void command::clear()
{
subCommands.clear();
doubles.clear();
integers.clear();
}

/*
This function is used by the priority queue to sort the order of the commands in the queue.  It less the < operator be used to compare the priority of two commands.
@param inputLeftCommand: The command on the left side of the < symbol
@param inputRightCommand: The command on the right side of the < symbol
@return: True if the left command is less than the right command
*/
bool operator< (const command &inputLeftCommand, const command &inputRightCommand)
{
return inputLeftCommand.priority < inputRightCommand.priority;
}
