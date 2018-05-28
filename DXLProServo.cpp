//#include <iostream>

using namespace std;

/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*///////////////////////////////////////////////////////////////////////////////
Created by: Sayyed Omar Kamal
Date: 29 March 2018

For Dynamixel MX-64 Servos.
*////////////////////////////////////////////////////////////////////////////////

#include "DXLServo.h"

// If getch() needed. getch call for Windows is _getch(), for non-Windows, need to use below code
int getch(void)
{
//#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
//#elif defined(_WIN32) || defined(_WIN64)
//    return _getch();
//#endif
}

// if kbhit() needed
/*
int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}
*/

////////////////////////////////////////////////////   DXLServo class definition   /////////////////////////////////////////////////////////////////////////////////////////

DXLServo::DXLServo() {							// Constructor
    identity = 1;								// default ID 1
    protocolVersion = 2.0;						// default Protocol 2.0
    baudRate = 57600;							// default 57,600 baud

    goalPositionVector.clear();					// Initialize empty vector at start
    present_position = 0, present_current = 0, present_temperature = 25;				// Basic starting values, change with read later
    limitAccel = 1, limitVel = 1, limitCurrent = 0, limitPosMin = 0, limitPosMax = 4095;
    profileAccel = 1, profileVel = 1;

    dxl_comm_result = COMM_TX_FAIL;				// Initial comm result
    dxl_error = 0;								// Dynamixel error
    //deviceName = std::string( "COM" );
}

DXLServo::~DXLServo() {							// Destructor

}

/*void setDeviceName(int deviceNum) {
    char numstr[21]; // enough to hold all numbers up to 64-bits
    sprintf_s(numstr, "%f", deviceNum);
    //std::string deviceName = std::string( "COM" );
    deviceName = deviceName + numstr;
}*/

/*void DXLServo::initPortHandler() {				// Initialize PortHandler instance, port path, etc
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler( deviceName.c_str() );
}*/

/*void DXLServo::initPacketHandler() {			// Initialize PacketHandler instance, protocol version, etc
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler( protocolVersion );
}*/

void DXLServo::enableTorque() {			// Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, identity, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf( "%s\n", packetHandler->getTxRxResult( dxl_comm_result ) );
    }
    else if (dxl_error != 0)
    {
        printf( "%s\n", packetHandler->getRxPacketError( dxl_error ) );
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", identity);
    }
}

void DXLServo::disableTorque() {			// Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, identity, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void DXLServo::addToGoalPosVector( std::vector<int> posVector ) {			// Store a vector of preselected Goal Positions into internal vector
    for (int i = 0; i < posVector.size(); i++) {
        goalPositionVector.push_back( posVector[i] );
    }
}

int DXLServo::checkGoalPosVector() {
    if ( goalPositionVector.size() == 0 ) {
        printf("Error! Goal Position Vector is empty!\n");
        return -1;
    }
    else {
        bool check = false;
        for ( int i = 0; i < goalPositionVector.size(); i++ ) {
            if ( goalPositionVector[i] < 0 || goalPositionVector[i] > 4095) {
                printf("Error! Goal Position Vector contains invalid position at index %d!\n", i);
                printf("!!! Modify or reinitialize Position Vector !!!\n");
                check = true;
                break;			// Exit for loop at any point error occurs
            }
        }
        if (check) return 0;
        else return 1;
    }
}

void DXLServo::writeGoalPosition( int vectorPosition) {						// Write DXL position from internal stored vector
    if ( goalPositionVector.size() == 0 ) {				// If array is empty
        printf( "Error! Goal Position Vector is empty!\n" );
    }
    else {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_GOAL_POSITION, goalPositionVector[vectorPosition], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    }
}

void DXLServo::writeGoalPosition(int select, int vectorPosition) {
    if ( select == 0 ) {			// Internal Position Vector
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_GOAL_POSITION, goalPositionVector[vectorPosition], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    }
    else if ( select == 1 ) {		// Direct entry
        if ( vectorPosition >= 0 && vectorPosition < 4096) {
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_GOAL_POSITION, vectorPosition, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }
        }
        else {
            printf("Error! Invalid value of desired Goal Position; Values between 0 - 4095 only, from -180 to +180 degrees in units of 0.088 degrees!\n");
        }
    }
    else {
        printf("Error! Invalid Select value; 0 for direct entry or 1 for internal reference!\n");
    }
}

int DXLServo::readCurrentPosition() {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, identity, ADDR_MX_PRESENT_POSITION, (uint32_t*)&present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf( "%s\n", packetHandler->getTxRxResult( dxl_comm_result ) );
    }
    else if (dxl_error != 0)
    {
        printf( "%s\n", packetHandler->getRxPacketError( dxl_error ) );
    }
    return present_position;
}

int DXLServo::checkOperating() {
    uint8_t set_operate_mode = -1;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, identity, ADDR_MX_OPERATING_MODE, (uint8_t*)&set_operate_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    if (set_operate_mode < 0) {
        printf("Error! Operate Mode value is negative!\n");
    }
    else return set_operate_mode;
}

int DXLServo::checkPositionMode() {
    uint8_t set_operate_mode = -1;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, identity, ADDR_MX_OPERATING_MODE, (uint8_t*)&set_operate_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    if (set_operate_mode < 0) {
        printf("Error! Operate Mode value is negative! Unexpected error!\n");
        return -1;
    }
    else if (set_operate_mode != 3) {
        printf("Warning! Operate Mode not set to Position Control!\n");
        return 0;
    }
    else return 1;
}

void DXLServo::setOperatingMode(int mode) {
    uint8_t opMode = 3;									// Default Operating Mode: 3, Position Control
    switch (mode) {
    case 0:	opMode = DXL_CURRENT_CONTROL_MODE;
        break;
    case 1: opMode = DXL_VELOCITY_CONTROL_MODE;
        break;
    case 2: opMode = DXL_POSITION_CONTROL_MODE;
        break;
    case 3: opMode = DXL_EXTENDED_CONTROL_MODE;
        break;
    case 4: opMode = DXL_CURRENT_POSITION_CONTROL_MODE;
        break;
    case 5: opMode = DXL_PWM_CONTROL_MODE;
        break;
    }
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, identity, ADDR_MX_OPERATING_MODE, opMode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void DXLServo::set4ACurrentLimit() {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, identity, ADDR_MX_CURRENT_LIMIT, DXL_CURRENT_LIMIT_4A, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitCurrent = DXL_CURRENT_LIMIT_4A;
}

void DXLServo::setCurrentAmpsLimit(double amps) {
    if (amps > 4.0 || amps < 0) {
        printf("Error! Invalid value of current limit! Select between 0.0 and 4.0!\n");
        return;
    }

    int limit = int( int( (amps / 0.00336) + 0.5 ) );								// Rounding number trick; int(float value + 0.5)

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, identity, ADDR_MX_CURRENT_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitCurrent = limit;
}

void DXLServo::setCurrentAmpsLimit(int limit) {
    if (limit > 1941 || limit < 0) {
        printf("Error! Invalid value of current limit! Select between 0 and 1941!\n");
        return;
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, identity, ADDR_MX_CURRENT_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitCurrent = limit;
}

void DXLServo::set80rpmVelLimit() {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_VELOCITY_LIMIT, DXL_VELOCITY_LIMIT_80, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitVel = DXL_VELOCITY_LIMIT_80;
}

void DXLServo::setVelocityLimit(int limit) {
    if (limit > 1023 || limit < 0) {
        printf("Error! Invalid value of velocity limit! Select between 0 and 1023!\n");
        return;
    }
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_VELOCITY_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitVel = limit;
}

void DXLServo::setlowAccelLimit() {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_ACCELERATION_LIMIT, DXL_ACCELERATION_LIMIT_LOW, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitAccel = DXL_ACCELERATION_LIMIT_LOW;
}

void DXLServo::setAccelLimit(int limit) {
    if ( limit > 100 || limit < 0) {
        printf("Error! Invalid value of acceleration limit! Select between 0 and 100!/n100 value limit for safe operation!\n");
        return;
    }
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_ACCELERATION_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    limitAccel = limit;
}

void DXLServo::setPositionLimit(bool minMax, int angle) {
    int address = 0;
    if ( !minMax ) {									// minMax == false, Min Position Limit
        address = ADDR_MX_MIN_POSITION_LIMIT;
    }
    else address = ADDR_MX_MAX_POSITION_LIMIT;			// minMax == true, Max Position Limit

    int limit = int( ( double(angle + 180)/0.088 ) + 0.5 );

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    if (!minMax)	limitPosMin = limit;
    else			limitPosMax = limit;
}

void DXLServo::setProfileAcceleration(int accel) {
    if (accel == 0) {
        printf("Warning! Value of 0 will give infinite acceleration! Disallowed for safe operation!\n");
        return;
    }
    else if (accel > limitAccel || accel < 0) {
        printf("Error! Invalid value! Select value between 1 - %d!\n", limitAccel);
        return;
    }

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_PROFILE_ACCELERATION, accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    profileAccel = accel;
}

int DXLServo::checkProfileAcceleration() {
    int accel = -1;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, identity, ADDR_MX_PROFILE_ACCELERATION, (uint32_t*)&accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    if (accel < 0) {
        printf("Error! Value is negative! Unexpected error!\n");
        return -1;
    }
    else return accel;
}

void DXLServo::setProfileVelocity(int vel) {
    if (vel == 0) {
        printf("Warning! Value of 0 will give infinite velocity! Disallowed for safe operation!\n");
        return;
    }
    else if (vel > limitVel || vel < 0) {
        printf("Error! Invalid value! Select value between 1 - %d!\n", limitVel);
        return;
    }

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, identity, ADDR_MX_PROFILE_ACCELERATION, vel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    profileVel = vel;
}

int DXLServo::checkProfileVelocity() {
    int vel = -1;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, identity, ADDR_MX_PROFILE_VELOCITY, (uint32_t*)&vel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    if (vel < 0) {
        printf("Error! Value is negative! Unexpected error!\n");
        return -1;
    }
    else return vel;
}

int DXLServo::checkPresentTemperature() {
    int limitTemp = -1;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, identity, ADDR_MX_TEMPERATURE_LIMIT, (uint8_t*)limitTemp, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    if (limitTemp < 0) {					// Error in comm function return, unexpected error
        printf("Error! Value (EEPROM Limit) is negative! Unexpected error!\n");
        return -1;
    }
    else {									// Return value should be 80 with Temp Limit EEPROM value unchanged
        int presTemp = -1;
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, identity, ADDR_MX_PRESENT_TEMPERATURE, (uint8_t*)presTemp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        if (presTemp < 0) {
            printf("Error! Value (Presnt Temperature) is negative! Unexpected error!\n");
            return -1;
        }
        else if ( presTemp >= (0.9 * limitTemp) && presTemp < limitTemp ) {			// If Present Temperature is around 90% of limit or higher but lower than limit
            printf("Warning! Temperature has reached 90% of limit!\n");
            return 1;
        }
        else if (presTemp >= limitTemp) {					// If Present Temperature is at or exceeds limit ( 80 - 100 C )
            printf("Error! Temperature has exceeded limit! Shutdown triggering!\n");
            return 2;
        }
    }
    return 0;
}

int DXLServo::getPresentTemperature() {
    int presTemp = -1;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, identity, ADDR_MX_PRESENT_TEMPERATURE, (uint8_t*)presTemp, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    if (presTemp < 0 || presTemp > 100) {							// Present Temperature valid range: 0 - 100, outside range -> Error
        printf("Error! Value is outside valid range! Unexpected error!\n");
        return -1;
    }
    return presTemp;			// Temperature value ~= Actual Temperature, 1:1 relation
}

int DXLServo::checkPresentCurrent() {
    if (limitCurrent <= 0) {													// Limit should not be 0 or negative
        printf("Error! Limit has not been set or unexpected error has occurred!\n");
        return -1;
    }
    else {
        int presCurr = -1;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, identity, ADDR_MX_PRESENT_CURRENT, (uint16_t*)presCurr, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        if (presCurr < 0) {
            printf("Error! Value is negative! Unexpected Error!\n");
            return -1;
        }
        else if (presCurr >= (0.9*limitCurrent) && presCurr < limitCurrent) {
            printf("Warning! Current is close to limit!\n");
            return 1;
        }
        else if (presCurr >= limitCurrent) {
            printf("Error! Current has exceeded limit!\n");
            return 2;
        }
    }
    return 0;
}

int DXLServo::getPresentCurrent() {
    int presCurr = -1;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, identity, ADDR_MX_PRESENT_CURRENT, (uint16_t*)presCurr, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    if (presCurr < 0 || presCurr > 1941) {										// Present Current valid range: 0 - 1941
        printf("Error! Value is outside valid range! Unexpected Error!\n");
        return -1;
    }
    //presCurr = presCurr * 0.00336;												// Present Current value * 3.36mA = Actual Current {uncomment if conversion wanted}
    return presCurr;
}

////////////////////////////////////////////////////   End of DXLServo class   /////////////////////////////////////////////////////////////////////////////////////////////

///*
////////////////////
//		29 Mar 2018 changes not yet implemented in example.
//      21 May: Linux test
////////////////////

////////////////////////////////////////////////////    Example Code Using DXLServo functions    ///////////////////////////////////////////////////////////////////////////
int main() {
    // Initialize Servo objects and parameters

    DXLServo panServo;								// DXLServo object 1
    if ( panServo.getDXLID() != 1 ) {				// Ensure Pan set to ID 1
        panServo.setDXLID(1);
    }
    panServo.setDeviceName(0);						// USB port ttyUSB0
    //panServo.initPortHandler();
    //panServo.initPacketHandler();					// Initialize Porthandler and Packethandler objects
    /*
    std::vector<int> positions;						// Initialize vector
    positions.clear();								// Clear, ensure no erratic values
    int startPos = 1024;
    for (int a = 0; a < 5; a++) {					// Push preset Pan positions into vector
        positions.push_back(startPos);				// Pan positions {-90, -45, 0, 45, 90}
        startPos += 512;
    }
    panServo.addToGoalPosVector(positions);			// Push positions into internal Vector
    positions.clear();								// Clear vector for Tilt next
    */
    DXLServo tiltServo;								// DXLServo object 2
    if (tiltServo.getDXLID() != 2) {				// Ensure Tilt set to ID 2
        tiltServo.setDXLID(2);
    }
    tiltServo.setDeviceName(0);						// USB port ttyUSB0
    //tiltServo.initPortHandler();
    //tiltServo.initPacketHandler();

    /*
    positions.push_back(1538);
    positions.push_back(1877);
    positions.push_back(2218);
    positions.push_back(2560);
    positions.push_back(2901);
    positions.push_back(3072);						// Tilt positions {-45, -15, 15, 45, 75, 90}
    tiltServo.addToGoalPosVector(positions);		// Push Tilt positions into internal Vector
    positions.clear();								// Clear vector, free up memory
    */

    // Begin DXL operations
    // Open Comms ports
    //panServo.openPort();
    //tiltServo.openPort();
    // Set port Baud rate using internal baudRate data
    //panServo.setPortBaudRate();
    //tiltServo.setPortBaudRate();

    //int fd = 0;
    //fd = open(panServo.deviceName.c_str(), O_CREAT | O_RDWR | O_NOCTTY | O_NDELAY);
    //fd = open(panServo.deviceName.c_str(), O_RDWR | O_NOCTTY);
    //if(fd == -1)    cout << "unable to open" << endl;

    FILE* serialPort;
    serialPort = fopen("/dev/ttyUSB0", "r+");  //"/dev/ttyUSB0"
    if(serialPort == NULL)
    {
        cout << "Error opening file." << endl;
        return(0);
    }

    //init
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(panServo.deviceName.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
    // Open Port
    if (portHandler->openPort())
      {
        printf("Succeeded to open the port!\n");
      }
      else
      {
        printf("Failed to open the port!\n");
        return 0;
      }
    // Set port baudrate
      if (portHandler->setBaudRate(57600))
      {
        printf("Succeeded to change the baudrate!\n");
      }
      else
      {
        printf("Failed to change the baudrate!\n");
        return 0;
      }

    // Enable Torque outputs
    panServo.enableTorque();
    tiltServo.enableTorque();

    int pan = 0, tilt = 0;
    int currentPanPos = 0, currentTiltPos = 0, currentPanTemp = 0, currentTiltTemp = 0;

    while (1) {
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE) break;

       // std::cout << "Hello.\nPress enter to continue." << std::endl;
        std::cin.get();

        // Read current positions
        currentPanPos = panServo.readCurrentPosition();
        currentTiltPos = tiltServo.readCurrentPosition();

        currentPanTemp = panServo.getPresentTemperature();
        currentTiltTemp = tiltServo.getPresentTemperature();


        /*
        // Write new positions
        panServo.writeGoalPosition(pan);
        tiltServo.writeGoalPosition(tilt);

        // Wait for Servos move to new positions
        int tempPanGoal = panServo.getCurrentGoal(pan);
        int tempTiltGoal = tiltServo.getCurrentGoal(tilt);
        while ( abs(tempPanGoal - currentPanPos) > DXL_MOVING_STATUS_THRESHOLD || abs(tempTiltGoal - currentTiltPos) > DXL_MOVING_STATUS_THRESHOLD ) {
            currentPanPos = panServo.readCurrentPosition();
            currentTiltPos = tiltServo.readCurrentPosition();

            // Print line stating move status of both motors
            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", panServo.getDXLID(), tempPanGoal, currentPanPos, tiltServo.getDXLID(), tempTiltGoal, currentTiltPos);
        }

        // Change Goal Position reference index for next loop
        pan++;
        if (pan == 5) pan = 0;
        tilt++;
        if (tilt == 6) tilt = 0;
        */
    }

    // Disable DXLs Torque outputs
    panServo.disableTorque();
    tiltServo.disableTorque();

    portHandler->closePort();
    //close(fd);
    // Close Comms port
    //panServo.closePort();
    //tiltServo.closePort();

    return 0;
}
// End of C++ Example Code using DXLServo functions
//*/

// C Style code of SDK functions
/*int main()
{
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler1 = dynamixel::PortHandler::getPortHandler(DEVICENAME1);
    dynamixel::PortHandler *portHandler2 = dynamixel::PortHandler::getPortHandler(DEVICENAME2);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index1 = 0, index2 = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result


    //CHANGE THIS WHEN TESTING ON QUICABOT!
    //int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position
    int dxl1_goal_position[5] = { 1024, 1536, 2048, 2560, 3072 };         // Goal position for Pan
    int dxl2_goal_position[6] = { 1536, 1875, 2216, 2560, 2898, 3072 };         // Goal position for Tilt
    //CHANGE THIS WHEN TESTING ON QUICABOT!

    uint8_t dxl_error = 0;                          // Dynamixel error
    int32_t dxl1_present_position = 0;               // Present position: Pan
    int32_t dxl2_present_position = 0;               // Present position: Tilt
    int32_t dxl1_present_current = 0;               // Present current: Pan
    int32_t dxl2_present_current = 0;               // Present current: Tilt
    int32_t dxl1_present_temperature = 0;               // Present temperature: Pan
    int32_t dxl2_present_temperature = 0;               // Present temperature: Tilt


    // Open port 1
    if (portHandler1->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        _getch();
        return 0;
    }
    // Open port 2
    if (portHandler2->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        _getch();
        return 0;
    }

    // Set port baudrate for motor 1
    if (portHandler1->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        _getch();
        return 0;
    }
    // Set port baudrate for motor 2
    if (portHandler2->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        _getch();
        return 0;
    }

    // Enable Dynamixel Torque for DXL 1
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL_ID1);
    }
    // Enable Dynamixel Torque for DXL 2
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL_ID2);
    }

    while (1)
    {
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (_getch() == ESC_ASCII_VALUE)
            break;

        // Write goal position to DXL1
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler1, DXL_ID1, ADDR_MX_GOAL_POSITION, dxl1_goal_position[index1], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        // Write goal position to DXL2
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, DXL_ID2, ADDR_MX_GOAL_POSITION, dxl2_goal_position[index2], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do
        {
            // Read present position of DXL1
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler1, DXL_ID1, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }
            // Read present position of DXL2
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler2, DXL_ID2, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID1, dxl1_goal_position[index1], dxl1_present_position, DXL_ID2, dxl2_goal_position[index2], dxl2_present_position);

        } while ((abs(dxl1_goal_position[index1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl2_goal_position[index2] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Change goal position: Pan
        if ( index1 >= 0 && index1 < ( sizeof dxl1_goal_position / sizeof *dxl1_goal_position ) )
        {
            index1 += 1;
        }
        else
        {
            index1 = 0;
        }
        // Change goal position: Tilt
        if ( index2 >= 0 && index2 < (sizeof dxl2_goal_position / sizeof *dxl2_goal_position))
        {
            index2 += 1;
        }
        else
        {
            index2 = 0;
        }
    }

    // Disable Dynamixel Torqueof DXL1
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    // Disable Dynamixel Torqueof DXL2
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close port DXL1
    portHandler1->closePort();
    // Close port DXL2
    portHandler2->closePort();

    return 0;
}
// End of C style code
*/
