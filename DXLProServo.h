#pragma once

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <algorithm>
#include <vector>
#include <string>
//#include <iostream>
#include <sstream>

#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
//#include <time.h> 
//#include <chrono>

#include "dynamixel_sdk/dynamixel_sdk.h"                                // Uses Dynamixel SDK library

// Control table address
//EEPROM
#define ADDR_PRO_ID						7	
#define ADDR_PRO_BAUD_RATE				8
#define ADDR_PRO_OPERATING_MODE			11
#define ADDR_PRO_TEMPERATURE_LIMIT		21

//#define ADDR_MX_CURRENT_LIMIT			38	//no current limit in Pro versions, use torque limit

#define ADDR_PRO_ACCELERATION_LIMIT		26
#define ADDR_PRO_TORQUE_LIMIT			30
#define ADDR_PRO_VELOCITY_LIMIT			32
#define ADDR_PRO_MAX_POSITION_LIMIT		36
#define ADDR_PRO_MIN_POSITION_LIMIT		40
#define ADDR_PRO_EXT_PORT_MODE_1		44
#define ADDR_PRO_EXT_PORT_MODE_2		45
#define ADDR_PRO_EXT_PORT_MODE_3		46
#define ADDR_PRO_EXT_PORT_MODE_4		47
#define ADDR_PRO_SHUTDOWN				48
//RAM
#define ADDR_PRO_TORQUE_ENABLE			562
#define ADDR_PRO_LED_RED				563
#define ADDR_PRO_LED_GREEN				564
#define ADDR_PRO_LED_BLUE				565

//#define ADDR_MX_PROFILE_ACCELERATION	108
//#define ADDR_MX_PROFILE_VELOCITY		112		//Pro versions do not use profile vel/acc. Need to confirm once servo arrives

#define ADDR_PRO_VELOCITY_I_GAIN		586
#define ADDR_PRO_VELOCITY_P_GAIN		588
#define ADDR_PRO_POSITION_P_GAIN		594
#define ADDR_PRO_GOAL_POSITION			596
#define ADDR_PRO_GOAL_VELOCITY			600
#define ADDR_PRO_GOAL_TORQUE			604
#define ADDR_PRO_GOAL_ACCELERATION		606
#define ADDR_PRO_MOVING					610
#define ADDR_PRO_PRESENT_POSITION		611
#define ADDR_PRO_PRESENT_VELOCITY		615
#define ADDR_PRO_PRESENT_CURRENT		621
#define ADDR_PRO_PRESENT_TEMPERATURE	625
#define ADDR_PRO_EXT_PORT_DATA_1		626
#define ADDR_PRO_EXT_PORT_DATA_2		628
#define ADDR_PRO_EXT_PORT_DATA_3		630
#define ADDR_PRO_EXT_PORT_DATA_4		632
#define ADDR_PRO_REGISTERED_INSTRUCTION	890
#define ADDR_PRO_STATUS_RETURN_LEVEL	891
#define ADDR_PRO_HARDWARE_ERROR_STATUS	892

// Not used in Pro versions
// Protocol version
//#define PROTOCOL_VERSION                2.0                 // Using Protocol 2.0

// Default setting
//Servo specific values
#define DXL_ID1								1                  // Dynamixel1 ID, Pan Servo
#define DXL_ID2								2                  // Dynamixel2 ID, Tilt Servo
#define DXL_BAUDRATE_57600					1                  // Baudrate set value for 57600 baud, default baudrate
#define DXL_PRO_TORQUE_CONTROL_MODE			0					// Operating Mode set value for Torque Control Mode, no velocity/position
#define DXL_PRO_VELOCITY_CONTROL_MODE		1					// Operating Mode set value for Velocity Control Mode, no position
#define DXL_PRO_POSITION_CONTROL_MODE		3					// Operating Mode set value for Position Control (Single turn) Mode, all fields controllable
#define DXL_PRO_EXTENDED_CONTROL_MODE		4					// Operating Mode set value for Extended Position Control (Multi turn) Mode

//#define DXL_CURRENT_LIMIT_4A				1190				// Current Limit set value for 4 Amps maximum, MX-64R stall current: 4.1A at 12V || No current control for Pro versions

#define DXL_PRO_ACCELERATION_LIMIT_LOW		26					// Acceleration Limit set value for 5227 rev/min2 maximum, based on 1% of velocity operation default
#define DXL_PRO_TORQUE_LIMIT_2A				497					// Torque Limit set value for 2002mA of current, based on performance graph
#define DXL_PRO_VELOCITY_LIMIT_80			20562				// Velocity Limit set value for 80 rpm maximum

//General servo data values
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_PRO_MOVING_THRESHOLD		50                  // Dynamixel moving threshold, default value
#define DXL_PRO_ABS_POSITION_MAX_VALUE		131593			//Absolute value of position data max for Pro M42; -ve for min, +ve for maximum

// Profile Vel/Acc not used in Pro versions
//#define PROFILE_ACCELERATION_MAX		15					//Limiting value to ensure DXL does not accelerate too quickly, causing following error
//#define PROFILE_VELOCITY_MAX			200					//Limiting value to ensure DXL does not turn too quickly, causing following error

//Values for system commands
#define BAUDRATE                        57600				//For use in code, not to set DXL Baudrate
//#define DEVICENAME1                      "COM1"				// Check which port is being used on each servo
//#define DEVICENAME2                      "COM2"				// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define ESC_ASCII_VALUE                 0x1b

class DXLServo {
private:
	double protocolVersion;
	std::vector<int> goalPositionVector;
	uint8_t dxl_error;
	int32_t present_position, present_temperature;
	//int dxl_comm_result, baudRate, identity, limitAccel, limitVel, profileAccel, profileVel, limitCurrent, limitPosMin, limitPosMax;

	int dxl_comm_result, baudRate, identity, limitAccel, limitTorque, limitVel, limitPosMin, limitPosMax;
	int extPortMode1, extPortMode2, extPortMode3, extPortMode4;

#if defined(__linux__) || defined(__APPLE__)
	std::string deviceName = std::string("/dev/ttyUSB");
#elif defined(_WIN32) || defined(_WIN64)
	std::string deviceName = std::string("COM");
#endif

	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;

protected:

public:
	DXLServo();
	~DXLServo();

	//std::string deviceName;
	//std::string setDeviceName(int deviceNum) {

	void setDXLID(int dxlNum) {
		identity = dxlNum;
	}

	int getDXLID() {								// get DXL ID set in code memory
		return identity;
	}

	void setDeviceName() {							// If default ID used, COM1 or /dev/ttyUSB1
		std::stringstream sstm;
		sstm << deviceName << 1;
		deviceName = sstm.str();
	}

	void setDeviceName(int deviceNum) {				// For multi DXL ID
		std::stringstream sstm;
		sstm << deviceName << deviceNum;
		deviceName = sstm.str();
	}

	void initPortHandler() {
		dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
	}

	void setProtocolVersion(double protocolNum) {				// Set Protocol 1.0 or 2.0
		protocolVersion = protocolNum;
	}

	void initPacketHandler() {
		dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(protocolVersion);
	}

	void openPort() {			// Open comms port to DXL
		if (portHandler->openPort())
		{
			printf("Succeeded to open the port!\n");
		}
		else
		{
			printf("Failed to open the port!\n");
			//printf("Press any key to terminate...\n");
			//_getch();
		}
	}

	void setDeviceBaudRate(int desiredRate) {
		baudRate = desiredRate;
	}

	void closePort() {
		portHandler->closePort();
	}

	void setPortBaudRate() {
		if (portHandler->setBaudRate(baudRate))
		{
			printf("Succeeded to change the baudrate!\n");
		}
		else
		{
			printf("Failed to change the baudrate!\n");
			//printf("Press any key to terminate...\n");
			//_getch();
			//return 0;
		}
	}
	void enableTorque();
	void disableTorque();

	void addToGoalPosVector(std::vector<int> posVector);
	int checkGoalPosVector();												// Check internal Position Vector for empty vector or invalid values
	void resetGoalPosVector() {												// Reset internal Position Vector for new values if needed
		goalPositionVector.clear();
	}
	void writeGoalPosition(int vectorPosition);							// Write Goal Position to DXL from inetrnal GoalPos Vector
	void writeGoalPosition(int select, int vectorPosition);				// Write Goal Position to DXL directly or from internal Goal Position Vector; int select = {0 -> from internal vector, 1 -> direct}
	int readCurrentPosition();
	int getCurrentGoal(int index) {
		return goalPositionVector[index];
	}

	int checkOperating();									// Return value of Operating Mode; 0 = Torque control, 1 = Velocity control, 3 = Position control (single turn), 4 = Extended Position control (Multi-turn)
	int checkPositionMode();								// Check if Operating Mode is set to Position Control (Single Turn). Returns 1 if yes, 0 if other control modes, -1 if error.
	void setOperatingMode(int mode);						// Change DXL Operating Mode; mode selection values 0-3, 0 = Torque, 1 = Velocity, 2 = Position (single turn), 3 = Extended Position(Multi turn)

	//// Current control not implemented in Pro versions
	//void set4ACurrentLimit();								// Set current limit to 4 Amps, value predefined in header
	//void setCurrentAmpsLimit(double amps);					// Set current limit between 0 - 4 amps. Limit = int(amps / 3.36mA)
	//void setCurrentAmpsLimit(int limit);					// Set current limit between 0 - 1941, current = limit * 3.36mA {externally calculated}

	void set80rpmVelLimit();								// Set Velocity limit to 80.15 rpm, value predefined in header
	void setVelocityLimit(int limit);						// Set Velocity limit between 0 - 38553, velocity = value * 0.00389076rpm {externally calculated}; 38553 is 150rpm, actual ProM42 velocity value limit is 2,147,483,647 => ~8 million rpm; limit value down to 38553 for safe operation

	void setlowAccelLimit();								// Set Acceleration limit to 26 * 201.039 = 5227 rev/min2, value predefined in header
	void setAccelLimit(int limit);							// Set Acceleration limit between 0 - 50, accel = limit * 201.039rev/min2 {externally calculated}, actual limit = 2,147,483,647 but limited to 50 as 431x10^9 rev/min2 is far too large accel for safe operation. 50 = 10000rpm2

	void setTorqueLimit();									// Set Torque limit to 2000mA; Pro uses current as torque, max value allowed: 32,767, formula: current(mA) = presentCurrent * 8250 / 2048. At max value, current = 131,000mA, unsafe. Limit value down to '496' for 2000mA
	

	void setPositionLimit(bool minMax, int angle);			// Set Position Limit; minMax: false for Min (Lower) limit, true for Max (Upper) limit; angle between -180 and +180 degrees

	////Profile Accel and Vel not implemented in Pro versions
	//void setProfileAcceleration(int accel);					// Set Profile Acceleration, accel must be between 1 and limit set in EEPROM, 0 not allowed so no infinite acceleration
	//int checkProfileAcceleration();							// Check value of Profile Acceleration in DXL RAM
	//void setProfileVelocity(int vel);						// Set Profile Velocity, vel must be between 1 and limit set in EEPROM, 0 not allowed so no infinite velocity
	//int checkProfileVelocity();								// Check value of Profile Velocity in DXL RAM

	int checkPresentTemperature();							// Check Present Temperature, compare against limit, warning at 10% from limit
	int getPresentTemperature();							// Return Presnt Temperature value, for displaying value externally

	int checkPresentCurrent();								// Check Present Current, compare against limit, warning at 10% from limit, error at/beyond limit
	int getPresentCurrent();								// Return Present Current value, for displaying externally

	void setLEDBrightness(int color, int bright);			// Set brightness of LEDs: color value to select Red (0), Green (1) or Blue(2); bright value for intensity, 0 - 255

	void setExternalPortMode(int portNum, int mode);		// Set operating mode of External Ports 1-4: portNum value to select port number 1 - 4; mode value to set operation: 0 = analog input mode, 1 = output mode, 2 = pull-up input mode, 3 = pull-up output mode. Currently, only using mode 1 output mode.
	void setExternalPortOutput(int portNum, int data);		// Set value of external port in output mode: portNum to select port 1 - 4; data value 0 (0V/GND) or 1 (3.3V).
};

