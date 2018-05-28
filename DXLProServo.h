#pragma once

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
//#include <time.h> 
//#include <chrono>

#include "dynamixel_sdk/dynamixel_sdk.h"                                // Uses Dynamixel SDK library

// Control table address
//EEPROM
#define ADDR_MX_ID						7	
#define ADDR_MX_BAUD_RATE				8
#define ADDR_MX_OPERATING_MODE			11
#define ADDR_MX_PROTOCOL_VERSION		13
#define ADDR_MX_TEMPERATURE_LIMIT		31
#define ADDR_MX_CURRENT_LIMIT			38
#define ADDR_MX_ACCELERATION_LIMIT		40
#define ADDR_MX_VELOCITY_LIMIT			44
#define ADDR_MX_MAX_POSITION_LIMIT		48
#define ADDR_MX_MIN_POSITION_LIMIT		52
//RAM
#define ADDR_MX_TORQUE_ENABLE			64	
#define ADDR_MX_HARDWARE_ERROR_STATUS	70
#define ADDR_MX_PROFILE_ACCELERATION	108
#define ADDR_MX_PROFILE_VELOCITY		112
#define ADDR_MX_GOAL_POSITION			116	
#define ADDR_MX_MOVING					122
#define ADDR_MX_MOVING_STATUS			123
#define ADDR_MX_PRESENT_CURRENT			126
#define ADDR_MX_PRESENT_POSITION		132
#define ADDR_MX_PRESENT_TEMPERATURE		146

// Protocol version
#define PROTOCOL_VERSION                2.0                 // Using Protocol 2.0

// Default setting
//Servo specific values
#define DXL_ID1								1                  // Dynamixel1 ID, Pan Servo
#define DXL_ID2								2                  // Dynamixel2 ID, Tilt Servo
#define DXL_BAUDRATE_57600					1                  // Baudrate set value for 57600 baud, default baudrate
#define DXL_CURRENT_CONTROL_MODE			0					// Operating Mode set value for Current Control Mode
#define DXL_VELOCITY_CONTROL_MODE			1					// Operating Mode set value for Velocity Control Mode
#define DXL_POSITION_CONTROL_MODE			3					// Operating Mode set value for Position Control (Single turn) Mode
#define DXL_EXTENDED_CONTROL_MODE			4					// Operating Mode set value for Extended Position Control (Multi turn) Mode
#define DXL_CURRENT_POSITION_CONTROL_MODE	5					// Operating Mode set value for Current-based Position Control Mode
#define DXL_PWM_CONTROL_MODE				16					// Operating Mode set value for PWM Control Mode

#define DXL_CURRENT_LIMIT_4A				1190				// Current Limit set value for 4 Amps maximum, MX-64R stall current: 4.1A at 12V
#define DXL_ACCELERATION_LIMIT_LOW			30					// Acceleration Limit set value for 6437 rev/min2 maximum
#define DXL_VELOCITY_LIMIT_80				350					// Velocity Limit set value for 80.15 rpm maximum
//#define DXL_POSITION_VALUE_N90				1023				// Position value for -90 degrees
//#define DXL_POSITION_VALUE_P90				3068				// Max move limit for Pan: +90
//#define DXL_POSITION_VALUE_N45				1534				// Min move limit for Tilt: -45

//General servo data values
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define PROFILE_ACCELERATION_MAX		15					//Limiting value to ensure DXL does not accelerate too quickly, causing following error
#define PROFILE_VELOCITY_MAX			200					//Limiting value to ensure DXL does not turn too quickly, causing following error
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

//Values for system commands
#define BAUDRATE                        57600				//For use in code, not to set DXL Baudrate
#define DEVICENAME1                      "COM1"				// Check which port is being used on each servo
#define DEVICENAME2                      "COM2"				// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define ESC_ASCII_VALUE                 0x1b

class DXLServo {
private:
	double protocolVersion;
	std::vector<int> goalPositionVector;
	uint8_t dxl_error;
	int32_t present_position, present_current, present_temperature;
	int dxl_comm_result, baudRate, identity, limitAccel, limitVel, profileAccel, profileVel, limitCurrent, limitPosMin, limitPosMax;
	
    //#if defined(__linux__) || defined(__APPLE__)
    //std::string deviceName = std::string( "/dev/ttyUSB" );
    //#elif defined(_WIN32) || defined(_WIN64)
    //std::string deviceName = std::string( "COM" );
    //#endif
	
	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;

protected:

public:
	DXLServo();
	~DXLServo();

    std::string deviceName = std::string( "/dev/ttyUSB" );

	//std::string deviceName;
	//std::string setDeviceName(int deviceNum) {

	void setDXLID( int dxlNum ) {
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

	void setDeviceBaudRate( int desiredRate ) {
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

	void addToGoalPosVector( std::vector<int> posVector );
	int checkGoalPosVector();												// Check internal Position Vector for empty vector or invalid values
	void resetGoalPosVector() {												// Reset internal Position Vector for new values if needed
		goalPositionVector.clear();
	}
	void writeGoalPosition( int vectorPosition);							// Write Goal Position to DXL from inetrnal GoalPos Vector
	void writeGoalPosition( int select, int vectorPosition );				// Write Goal Position to DXL directly or from internal Goal Position Vector; int select = {0 -> from internal vector, 1 -> direct}
	int readCurrentPosition();
	int getCurrentGoal(int index) {
		return goalPositionVector[index];
	}

	int checkOperating();									// Return value of Operating Mode; 0 = Current control, 1 = Velocity control, 3 = Position control (single turn), 4 = Extended Position control (Multi-turn), 5 = Current-based Position control, 16 = PWN control
	int checkPositionMode();								// Check if Operating Mode is set to Position Control (Single Turn). Returns 1 if yes, 0 if other control modes, -1 if error.
	void setOperatingMode(int mode);						// Change DXL Operating Mode; mode selection values 0-5, 0 = Current, 1 = Velocity, 2 = Position (single turn), 3 = Extended Position(Multi turn), 4 = Current based Position, 5 = PWM

	void set4ACurrentLimit();								// Set current limit to 4 Amps, value predefined in header
	void setCurrentAmpsLimit(double amps);					// Set current limit between 0 - 4 amps. Limit = int(amps / 3.36mA)
	void setCurrentAmpsLimit(int limit);					// Set current limit between 0 - 1941, current = limit * 3.36mA {externally calculated}

	void set80rpmVelLimit();								// Set Velocity limit to 80.15 rpm, value predefined in header
	void setVelocityLimit(int limit);						// Set Velocity limit between 0 - 1023, velocity = limit * 0.229rpm {externally calculated}

	void setlowAccelLimit();								// Set Acceleration limit to 30 * 214.577 = 6437 rev/min2, value predefined in header
	void setAccelLimit(int limit);							// Set Acceleration limit between 0 - 100, accel = limit * 214.577rev/min2 {externally calculated}, actual limit = 32767 but limited to 100 as 7million rev/min2 is far too large accel for safe operation.

	void setPositionLimit(bool minMax, int angle);			// Set Position Limit; minMax: false for Min (Lower) limit, true for Max (Upper) limit; angle between -180 and +180 degrees

	void setProfileAcceleration(int accel);					// Set Profile Acceleration, accel must be between 1 and limit set in EEPROM, 0 not allowed so no infinite acceleration
	int checkProfileAcceleration();							// Check value of Profile Acceleration in DXL RAM
	void setProfileVelocity(int vel);						// Set Profile Velocity, vel must be between 1 and limit set in EEPROM, 0 not allowed so no infinite velocity
	int checkProfileVelocity();								// Check value of Profile Velocity in DXL RAM

	int checkPresentTemperature();							// Check Present Temperature, compare against limit, warning at 10% from limit
	int getPresentTemperature();							// Return Presnt Temperature value, for displaying value externally

	int checkPresentCurrent();								// Check Present Current, compare against limit, warning at 10% from limit, error at/beyond limit
	int getPresentCurrent();								// Return Present Current value, for displaying externally
};
