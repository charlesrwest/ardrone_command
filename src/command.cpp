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
This function decomposes the command into a ROS message for sending over the network.  ROS messages don't support containing submessages of the same type as the message, so this requires all subcommands to be decomposed into a list of simple (nonrecursive) commands.
*/
ardrone_application_node::serialized_ardrone_command command::serialize()
{
//Unwrap the commands so that they can be converted into ros messages
std::vector<command> commandList;
unwrapCommand(*this, commandList); //Unwrap this command into the command list

//Serialize
ardrone_application_node::serialized_ardrone_command buffer;
buffer.command = serializeCommandPart(commandList[0]);

for(int i=1; i<commandList.size(); i++)
{
buffer.subcommands.push_back(serializeCommandPart(commandList[i]));
}

return buffer;
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
This function clears the command and then tells the drone to attempt to maintain its position at the given (specific) QR code defined location while it does other things (such as wait).  This is a QR code state estimation based command, which means that landing will automatically engage if the designated QR code has not been seen within SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING.
@param inputQRCodeID: The string identifying the QR code to use for state estimation (will use any if an empty string is passed)
@param inputXCoordinate: The x coordinate in the QR code coordinate system (meters)
@param inputYCoordinate: The y coordinate in the QR code coordinate system (meters)
@param inputZCoordinate: The z coordinate in the QR code coordinate system (meters)
*/
void command::setMaintainPositionAtSpecificQRCodePoint(const std::string &inputQRCodeID, double inputXCoordinate, double inputYCoordinate, double inputZCoordinate)
{
clear();
type = SET_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT;
strings.push_back(inputQRCodeID);
doubles.push_back(inputXCoordinate);
doubles.push_back(inputYCoordinate);
doubles.push_back(inputZCoordinate);
}

/*
This function clears the command and then tells the drone to stop trying to maintain/go to a specific point in the QR code coordinate system.
*/
void command::setCancelMaintainPositionAtSpecificQRCodePoint()
{
clear();
type = SET_CANCEL_MAINTAIN_POSITION_AT_SPECIFIC_QR_CODE_POINT;
}

/*
This function clears the command and then tells the drone to point at a specific QR code in its point that is defining its coordinate system.  This is a QR code state estimation based command, which means that landing will automatically engage if the designated QR code has not been seen within SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING.
@param inputQRCodeID: The string identifying the QR code to use for state estimation (will use any if an empty string is passed)
*/
void command::setMaintainOrientationTowardSpecificQRCode(const std::string &inputQRCodeID)
{
clear();
type = SET_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE;
strings.push_back(inputQRCodeID);
}

/*
This function clears the command and then tells the drone to stop trying to point at a specific QR code in its point that is defining its coordinate system.
*/
void command::setCancelMaintainOrientationTowardSpecificQRCode()
{
clear();
type = SET_CANCEL_MAINTAIN_ORIENTATION_TOWARD_SPECIFIC_QR_CODE;
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
This function clears the command and then tells the drone to maintain its current state for the given number of seconds until a specific QR code tag comes into its field of view.
@param inputQRCodeID: The string identifying the QR code to use for state estimation (will use any if an empty string is passed)
@param inputNumberOfSeconds: The number of seconds to wait
*/
void command::setWaitUntilSpecificQRCodeIsSpottedCommand(const std::string inputQRCodeID, double inputNumberOfSeconds)
{
clear();
type = SET_WAIT_UNTIL_SPECIFIC_QR_CODE_IS_SPOTTED_COMMAND;
strings.push_back(inputQRCodeID);
doubles.push_back(inputNumberOfSeconds);
}

/*
This function clears the command and then tells the drone to maintain its current state (seeking the QR code) for the given number of seconds until the weighted average position reaches within a certain distance of the target point.
@param inputNumberOfSeconds: The number of seconds to wait
@param inputTargetDistance: The distance from the point before the goal is considered achieved (in meters)
*/
void command::setWaitUntilPositionAtSpecificQRCodePointReachedCommand(double inputNumberOfSeconds, double inputTargetDistance)
{
clear();
type = SET_WAIT_UNTIL_POSITION_AT_SPECIFIC_QR_CODE_POINT_REACHED;
doubles.push_back(inputNumberOfSeconds);
doubles.push_back(inputTargetDistance);
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

/*
This function adds the command and then recursively goes through any subcommands in the command, adding them to the list of commands (depth first search).  The resulting list only contains commands without subcommands.  This is typically used to break down commands for serialization with the ROS message type.
@param inputCommand: The top command to process
@param inputCommandList: The list to start appending to
*/
void unwrapCommand(const command &inputCommand, std::vector<command> &inputCommandList)
{
//Add top level command without subcommands
command commandBuffer = inputCommand;
commandBuffer.subCommands.clear();

inputCommandList.push_back(commandBuffer);

//Process subcommands
for(int i=0; i<inputCommand.subCommands.size(); i++)
{
unwrapCommand(inputCommand.subCommands[i], inputCommandList);
}
}


/*
This function serializes the command to a serialized_ardrone_command_part, ignoring any subcommands the command may have.
@param inputCommand: The command to serialize
*/
ardrone_application_node::serialized_ardrone_command_part serializeCommandPart(const command &inputCommand)
{
ardrone_application_node::serialized_ardrone_command_part buffer;
buffer.type = inputCommand.type;
for(int i=0; i<inputCommand.strings.size(); i++)
{
buffer.strings.push_back(inputCommand.strings[i]);
}

for(int i=0; i<inputCommand.integers.size(); i++)
{
buffer.integers.push_back(inputCommand.integers[i]);
}

for(int i=0; i<inputCommand.doubles.size(); i++)
{
buffer.doubles.push_back(inputCommand.doubles[i]);
}

for(int i=0; i<inputCommand.flightAnimations.size(); i++)
{
buffer.flightAnimations.push_back(inputCommand.flightAnimations[i]);
}

for(int i=0; i<inputCommand.ledAnimations.size(); i++)
{
buffer.ledAnimations.push_back(inputCommand.ledAnimations[i]);
}

return buffer;
}

/*
This function converts a serialized command into a list of commands/subcommands.
@param inputSerializedCommand: The message to deserialize
@return: The list of commands stored in the message
*/
std::vector<command> deserialize_commands(const ardrone_application_node::serialized_ardrone_command &inputSerializedCommand)
{
std::vector<command> buffer;
buffer.push_back(deserialize_command_part(inputSerializedCommand.command));

for(int i=0; i<inputSerializedCommand.subcommands.size(); i++)
{
buffer.push_back(deserialize_command_part(inputSerializedCommand.subcommands[i]));
}

return buffer;
}

/*
This function converts a serialized command part into a command
@param inputSerializedCommandPart: The message to deserialize
@return: The command stored in the message
*/
command deserialize_command_part(const ardrone_application_node::serialized_ardrone_command_part &inputSerializedCommandPart)
{
command buffer;

buffer.type = (commandType) inputSerializedCommandPart.type;
for(int i=0; i<inputSerializedCommandPart.strings.size(); i++)
{
buffer.strings.push_back(inputSerializedCommandPart.strings[i]);
}

for(int i=0; i<inputSerializedCommandPart.integers.size(); i++)
{
buffer.integers.push_back(inputSerializedCommandPart.integers[i]);
}

for(int i=0; i<inputSerializedCommandPart.doubles.size(); i++)
{
buffer.doubles.push_back(inputSerializedCommandPart.doubles[i]);
}

for(int i=0; i<inputSerializedCommandPart.flightAnimations.size(); i++)
{
buffer.flightAnimations.push_back((flightAnimationType) inputSerializedCommandPart.flightAnimations[i]);
}

for(int i=0; i<inputSerializedCommandPart.ledAnimations.size(); i++)
{
buffer.ledAnimations.push_back((LEDAnimationType)  inputSerializedCommandPart.ledAnimations[i]);
}

return buffer;
}

