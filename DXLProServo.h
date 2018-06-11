#pragma once

/*///////////////////////////////////////////////////////////////////////////////
Created by: Sayyed Omar Kamal
Date: 8 June 2018

For Dynamixel Pro servos, based on MX-64 servo control. Basic C++ function calls working properly in Linux. 
*////////////////////////////////////////////////////////////////////////////////

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
#include <iomanip>
#include <sstream>
#include <cmath>

#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
//#include <time.h>
//#include <chrono>
//#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"                                // Uses Dynamixel SDK library

// Control table address
//EEPROM

#define ADDR_MX_ID						7
#define ADDR_PRO_ID                         7

#define ADDR_MX_BAUD_RATE				8
#define ADDR_PRO_BAUD_RATE                  8

#define ADDR_MX_OPERATING_MODE			11
#define ADDR_PRO_OPERATING_MODE             11

#define ADDR_MX_PROTOCOL_VERSION		13
// Pro servos fixed at Protocol 2.0

#define ADDR_MX_HOMING_OFFSET   		20
#define ADDR_PRO_HOMING_OFFSET              13

#define ADDR_MX_MOVING_THRESHOLD   		24
#define ADDR_PRO_MOVING_THRESHOLD           17

#define ADDR_MX_TEMPERATURE_LIMIT		31
#define ADDR_PRO_TEMPERATURE_LIMIT          21

// Voltage supplied with fixed dc-dc converter, no need to alter voltage limits
// MX only: PWM not in use for now, not adding

#define ADDR_MX_CURRENT_LIMIT			38
#define ADDR_PRO_TORQUE_LIMIT               30  //Pro uses Torque as current limit

#define ADDR_MX_ACCELERATION_LIMIT		40
#define ADDR_PRO_ACCELERATION_LIMIT         26

#define ADDR_MX_VELOCITY_LIMIT			44
#define ADDR_PRO_VELOCITY_LIMIT             32

#define ADDR_MX_MAX_POSITION_LIMIT		48
#define ADDR_MX_MIN_POSITION_LIMIT		52
#define ADDR_PRO_MAX_POSITION_LIMIT         36
#define ADDR_PRO_MIN_POSITION_LIMIT         40

// Pro only
#define ADDR_PRO_EXT_PORT_MODE_1            44
#define ADDR_PRO_EXT_PORT_MODE_2            45
#define ADDR_PRO_EXT_PORT_MODE_3            46
#define ADDR_PRO_EXT_PORT_MODE_4            47

#define ADDR_MX_SHUTDOWN    			63
#define ADDR_PRO_SHUTDOWN                   48

// Pro Indirect Address would be here, not in use at moment so not added

//RAM
#define ADDR_MX_TORQUE_ENABLE			64
#define ADDR_PRO_TORQUE_ENABLE              562

#define ADDR_MX_LED         			65
#define ADDR_PRO_LED_RED                    563
#define ADDR_PRO_LED_GREEN                  564
#define ADDR_PRO_LED_BLUE                   565

#define ADDR_MX_STATUS_RETURN_LEVEL		65
#define ADDR_PRO_STATUS_RETURN_LEVEL		891

// Read only
#define ADDR_MX_REGISTERED_INSTRUCTION  69
#define ADDR_PRO_REGISTERED_INSTRUCTION		890

// Read only
#define ADDR_MX_HARDWARE_ERROR_STATUS	70
#define ADDR_PRO_HARDWARE_ERROR_STATUS      892

// Velocity (I,P) Gains not implemented yet
// Position Gains (MX:D,I,P,FF2,FF1; Pro:P)
#define ADDR_MX_POSITION_D_GAIN			80
#define ADDR_MX_POSITION_I_GAIN			82
#define ADDR_MX_POSITION_P_GAIN			84
#define ADDR_MX_POSITION_FF2_GAIN		88
#define ADDR_MX_POSITION_FF1_GAIN		90
#define ADDR_PRO_POSITION_P_GAIN		    594

// MX only: Bus Watchdog not in use currently
// MX only: Goal PWM not in use currently

#define ADDR_MX_GOAL_CURRENT   			102
#define ADDR_PRO_GOAL_TORQUE                604

#define ADDR_MX_GOAL_VELOCITY  			104
#define ADDR_PRO_GOAL_VELOCITY              600

// Profile vars for MX only
#define ADDR_MX_PROFILE_ACCELERATION	108
#define ADDR_MX_PROFILE_VELOCITY		112

#define ADDR_MX_GOAL_POSITION			116
#define ADDR_PRO_GOAL_POSITION              596

// MX only: Realtime Tick not in use currently

#define ADDR_PRO_GOAL_ACCELERATION          606

// Read only
#define ADDR_MX_MOVING					122
#define ADDR_PRO_MOVING                     610

// MX only
#define ADDR_MX_MOVING_STATUS			123

// All Present Registers are read-only
#define ADDR_MX_PRESENT_CURRENT			126
#define ADDR_PRO_PRESENT_CURRENT			621

#define ADDR_MX_PRESENT_VELOCITY		128
#define ADDR_PRO_PRESENT_VELOCITY			615

#define ADDR_MX_PRESENT_POSITION		132
#define ADDR_PRO_PRESENT_POSITION			611

// MX only: Velocity and Position Trajectory not in use currently
// Present Input Voltage not in use currently

#define ADDR_MX_PRESENT_TEMPERATURE		146
#define ADDR_PRO_PRESENT_TEMPERATURE		625

// Pro only
#define ADDR_PRO_EXT_PORT_DATA_1            626
#define ADDR_PRO_EXT_PORT_DATA_2            628
#define ADDR_PRO_EXT_PORT_DATA_3            630
#define ADDR_PRO_EXT_PORT_DATA_4            632

// Indirect Data not in use currently

// Default settings
// Protocol version
#define PROTOCOL_VERSION                2.0                 // Using Protocol 2.0
//Servo specific values
#define DXL_ID1                                 1                   // Dynamixel1 ID, Pan Servo
#define DXL_ID2                                 2                   // Dynamixel2 ID, Tilt Servo
#define DXL_BAUDRATE_57600                      1                   // Baudrate set value for 57600 baud, default baudrate

// Operating Mode Values
#define DXL_MX_CURRENT_CONTROL_MODE             0                   // Operating Mode set value for Current Control Mode for MX
#define DXL_PRO_TORQUE_CONTROL_MODE             0                   // Operating Mode set value for Torque Control Mode for Pro
#define DXL_VELOCITY_CONTROL_MODE               1					// Operating Mode set value for Velocity Control Mode
#define DXL_POSITION_CONTROL_MODE               3					// Operating Mode set value for Position Control (Single turn) Mode
#define DXL_EXTENDED_CONTROL_MODE               4					// Operating Mode set value for Extended Position Control (Multi turn) Mode
#define DXL_MX_CURRENT_POSITION_CONTROL_MODE	5					// Operating Mode set value for Current-based Position Control Mode for MX only
#define DXL_MX_PWM_CONTROL_MODE                 16					// Operating Mode set value for PWM Control Mode for MX only

//Parameter Limits precalculated values
#define DXL_MX_CURRENT_LIMIT_MAX                 1190				// Current Limit set value for 4 Amps maximum, MX-64R stall current: 4.1A at 12V
#define DXL_MX_ACCELERATION_LIMIT_LOW			30					// Acceleration Limit set value for 6437 rev/min2 maximum, MX
#define DXL_MX_VELOCITY_LIMIT_80				350					// Velocity Limit set value for 80.15 rpm maximum, MX
#define DXL_PRO_M42_TORQUE_LIMIT_MAX                    521             // Torque limit for Pro M42, using current, absolute max at ~5.7Nm = 2.16A, use 2.1A
#define DXL_PRO_M42_VELOCITY_LIMIT_80                  20562            // Velocity limit for Pro M42, 80 rpm
#define DXL_PRO_M42_ACCEL_LIMIT_LOW                     30              // Acceleration limit for Pro M42, 6031 rpm2

//General servo data values
#define TORQUE_ENABLE                       1                   // Value for enabling the torque
#define TORQUE_DISABLE                      0                   // Value for disabling the torque
#define PROFILE_ACCELERATION_MAX            15					// Limiting value to ensure DXL does not accelerate too quickly, causing following error
#define PROFILE_MX_VELOCITY_MAX             200					// Limiting value to ensure DXL does not turn too quickly, causing following error
#define PROFILE_PRO_VELOCITY_MAX                2570
#define DXL_MX_MOVING_STATUS_THRESHOLD      10                  // Dynamixel moving status threshold
#define DXL_PRO_MOVING_STATUS_THRESHOLD         50

//Values for system commands
#define BAUDRATE                        57600				//For use in code, not to set DXL Baudrate
#if defined(__linux__) || defined(__APPLE__)
#define DEVICENAME1                      "/dev/ttyUSB1"
#elif defined(_WIN32) || defined(_WIN64)
#define DEVICENAME1                      "COM1"				// Check which port is being used on each servo
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#endif
#define ESC_ASCII_VALUE                 0x1b

//DXL Servo type defines
#define DXL_MX_64                 0
#define DXL_PRO_M42               1

class DXLServo {
private:
    std::vector<int> goalPositionVector;
    std::vector<double> goalAngleVector;
    int   limitAccel, limitVel, profileAccel, profileVel, limitCurrent, limitPosMin, limitPosMax, homeOffset;
    int externalPort[4];		// External Port Mode indicator. Ports 1 - 4, modes 0 - 3. E.g. externalPort[0] = 1; -> extrenal port 1 set to output mode.

protected:

public:
    DXLServo();
    ~DXLServo();

    double protocolVersion;
    uint8_t dxl_error;
    int dxl_comm_result, baudRate, identity, servoType;
    int present_position, present_temperature;
    double present_current;

    dynamixel::PortHandler *prtHandler;
    dynamixel::PacketHandler *pktHandler;

    //std::string deviceName = std::string( "/dev/ttyUSB" );
#if defined(__linux__) || defined(__APPLE__)
    std::string deviceName = std::string("/dev/ttyUSB");
#elif defined(_WIN32) || defined(_WIN64)
    std::string deviceName = std::string("COM");
#endif

    void setDXLServo(int servoModel) {
        if (servoModel == 0)		servoType = DXL_MX_64;
        else if (servoModel == 1)   servoType = DXL_PRO_M42;
        else    printf("Error! Invalid Servo selected! Select '0' for MX-64 or '1' for Pro M42!");
    }

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
        this->prtHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
    }

    void setProtocolVersion(double protocolNum) {				// Set Protocol 1.0 or 2.0
        protocolVersion = protocolNum;
    }

    void initPacketHandler() {
        this->pktHandler = dynamixel::PacketHandler::getPacketHandler(protocolVersion);
    }

    void openPort() {			// Open comms port to DXL
        if (this->prtHandler->openPort())
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
        this->prtHandler->closePort();
    }

    void setPortBaudRate() {
        if (this->prtHandler->setBaudRate(baudRate))
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
    void enableTorque();													// Enable servo motion. Must be set for servo to operate. Changes to EEPROM registers must be applied before calling this.
    void disableTorque();													// Disable servo motion. Call at end of use.

    // GoalPos functions use Position values for direct transmission
    void addToGoalPosVector(const std::vector<int> &posVector);					// Add Position values into internal vector. Uses std::vector<int> as input. CHANGE to int[], more common?
    //void addToGoalPosVector(int posVector[]);
    void addToGoalPosVector(int posVector) {									// Add 1 Position value to internal vector. Use in for loop with int array data.
        goalPositionVector.push_back(posVector);
    }
    int checkGoalPosVector();												// Check internal Position Vector for empty vector or invalid values
    void resetGoalPosVector() {												// Reset internal Position Vector for new values if needed
        goalPositionVector.clear();
    }
    void writeGoalPosition(int vectorPosition);							// Write Goal Position to DXL from internal GoalPos Vector
    void writeGoalPosition(int select, int vectorPosition);				// Write Goal Position to DXL directly or from internal Goal Position Vector; int select = {0 -> from internal vector, 1 -> direct}
    int readCurrentPosition();											// Read Position value from servo, position value does not include homing offset
    int getCurrentGoalPos(int index) {										// Read value of position in internal Goal Position vector, assumes user already knows size of vector
        return goalPositionVector[index];
    }

    // GoalAngle functions use angle as inputs, require conversion to Position values for transmission
    double readCurrentAngle();											// Read Position value and convert to angle, angle value includes homing offset, assumes initial read acquired
    void addToGoalAngleVect(const std::vector<double> &angVector);				// Add angle values into internal vector. Angle values depend on servo used (e.g. MX-64: 0-360 deg, Pro: -180 to 180).
    //void addToGoalAngleVect(double angVector[]);
    void addToGoalAngleVect(double angVector) {							// Add 1 Angle into internal vector. Use in for loop with double array data.
        goalAngleVector.push_back(angVector);
    }
    int checkGoalAngleVect();											// Check internal vector for empty vector
    void resetGoalAngleVect() {											// Reset internal Angle Vector for new values if needed
        goalAngleVector.clear();
    }
    void writeGoalAngle(int vectorAngle);								// Write Goal Angle to DXL from internal GoalAngle Vector
    void writeGoalAngle(int select, double vectorAngle);					// Write Goal Angle to DXL directly or from internal Goal Angle Vector; int select = {0 -> from internal vector, 1 -> direct}, conversion to Position value handled, includes home offset
    int getCurrentGoalAngle(int index) {								// Read value of angle in internal Goal Angle vector, assumes user already knows size of vector
        return goalAngleVector[index];
    }

    int checkOperating();									// Return value of Operating Mode; 0 = Current control, 1 = Velocity control, 3 = Position control (single turn), 4 = Extended Position control (Multi-turn), 5 = Current-based Position control, 16 = PWN control
    int checkPositionMode();								// Check if Operating Mode is set to Position Control (Single Turn). Returns 1 if yes, 0 if other control modes, -1 if error.
    void setOperatingMode(int mode);						// Change DXL Operating Mode; mode selection values 0-5, 0 = Current, 1 = Velocity, 2 = Position (single turn), 3 = Extended Position(Multi turn), 4 = Current based Position, 5 = PWM

    void setMaxCurrentLimit();								// Set current limit to max Amps before stall torque occurs, value predefined in header
    void setCurrentLimit(double amps);                      // Set current limit between 0 - servo limit. MX-64: 4.0 A, Pro M42: 0 - 2.1 A
    void setCurrentLimit(int limit);                        // Set current limit between 0 - 1941, current = limit * 3.36mA {externally calculated}

    void set80rpmVelLimit();								// Set Velocity limit to 80.15 rpm, value predefined in header
    void setVelocityLimit(int limit);						// Set Velocity limit between 0 - 1023, velocity = limit * 0.229rpm {externally calculated}
    void setVelocityLimit(double limit);					// Set Velocity limit between 0 - 235.0 rpm
    int getVelocityLimit();                                 // Return Velocity Limit set in DXL

    void setlowAccelLimit();								// Set Acceleration limit to 30 * 214.577 = 6437 rev/min2, value predefined in header
    void setAccelLimit(int limit);							// Set Acceleration limit between 0 - 100, limited to 100 as actual limit is far too large for safe operation.
    void setAccelLimit(double limit);						// Set Acceleration limit between 0 - (100 * unit), limited to 100 times for safe operation
    int getAccelLimit();                                    // Return Acceleration Limit set in DXL. Value multiply 214.517rev/mim2.

    void setPositionLimit(bool minMax, int position);		// Set Position Limit; minMax: false for Min (Lower) limit, true for Max (Upper) limit; Position between 0 and limit
    void setPositionLimit(bool minMax, double angle);		// Set Position Limit; minMax: false for Min (Lower) limit, true for Max (Upper) limit; angle between 0 to 360 or -180 and +180 degrees, see servo type

    void setProfileAcceleration(int accel);					// Set Profile Acceleration, accel must be between 1 and limit set in EEPROM, 0 not allowed so no infinite acceleration. Accel value in integer for direct transmission.
    void setProfileAcceleration(double accel);				// Set Profile Acceleration, accel must be between 1.0 and limit, 0 not allowed so no infinite acceleration. Acceleration in rev/min2 to be converted for transmission.
    int checkProfileAcceleration();							// Check value of Profile Acceleration in DXL RAM, return int, conversion available.
    void setProfileVelocity(int vel);						// Set Profile Velocity, vel must be between 1 and limit set in EEPROM, 0 not allowed so no infinite velocity, Velocity value in integer for direct transmission.
    void setProfileVelocity(double vel);					// Set Profile Velocity, vel must be between 1.0 and (limit * unit) set in EEPROM, 0 not allowed so no infinite velocity, Velocity in rpm to be converted for transmission
    int checkProfileVelocity();								// Check value of Profile Velocity in DXL RAM. Value multiply 0.229rpm

    int checkPresentTemperature();							// Check Present Temperature, compare against limit, issues warning at 10% from limit and at or above limit. Returns: -1 if negative error, 0 if no problem, 1 if close to limit, 2 if exceed limit.
    int getPresentTemperature();							// Return Present Temperature value, for displaying value externally

    int checkPresentCurrent();								// Check Present Current, compare against limit, warning at 10% from limit, error at/beyond limit. Returns: -1 if negative error, 0 if no problem, 1 if close to limit, 2 if exceed limit.
    double getPresentCurrent();								// Return Present Current value, for displaying externally

    int getHomingOffset();                                  // Return Homing Offset value

    bool isMoving();                                        // Check if servo is moving after write command

    int convertPostoVal(double angle);                      // Convert input angle (desired position) into Position value for transmission
    double convertValtoPos(int position);                   // Convert Position value from transmission into output angle

    int convertCurrtoVal(double current);					// Convert input current (amps) into Current value for transmission
    double convertValtoCurr(int value);						// Convert Current value from transmission into output current

    int convertVeltoVal(double velocity);					// Convert input velocity (rpm) into Velocity value for transmission
    double convertValtoVel(int value);						// Convert Velocity value from transmission into output velocity

    int convertAcctoVal(double accel);						// Convert input acceleration (rev/min2) into Acceleration value for transmission
    double convertValtoAcc(int value);						// Convert Acceleration value from transmission into output acceleration

    void servoReboot();										// Call when servo shuts down due to error to reactivate. Reactivates servo and prints error message with details of last occurred error.

    // Pro servos only
    void selectExtPortMode(int port, int mode);				// Select mode for External Ports 1-4, "port" to select port number, "mode" to select function. port: 1 - 4, mode: 0 - 3.
    void setExtPortData(int port, int data);				// For External Port output modes (1,3), set output to 0V or 3.3V.
    int readExtPortData(int port);							// Read value of External Port Datas.

    void  setLED(int color, int light);						// Activate or deactivate LED/s. MX servos, single LED. Pro servos, (r,g,b) LEDs. "color": 0 for MX single LED, (1,2,3) for Pro (Red, Green, Blue) LED. "light": MX: 0 off, 1 on; Pro: 0 - 255 intensity.

    void setPositionGain(int gainP, int gainI = 0, int gainD = 0, int gainF1 = 0, int gainF2 = 0);						// Set Position Gains for servo motors. Pro servos only have variable P gain, MX servos have (P,I,D,FF1,FF2)
    void readPositionGain(vector<int> &setGains);			// Read Position Gains currently set in registers, store in vector<int>. MX: 5 gain values. Pro: 1 gain value.

    int checkShutdown(int statusBits);				// Check Hardware Error Status register for Shutdown condition. If Shutdown detected and not Overheating Error, try reboot. If Overheating Error, disable servo, warn user to disconnect motor for at least half hour.

};

