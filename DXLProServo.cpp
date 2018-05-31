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
Date: 31 May 2018

For Dynamixel Pro servos, based on MX-64 servo control. Basic C++ function calls working properly in Linux. 
TOADD: Pro servo address and additional functions
*////////////////////////////////////////////////////////////////////////////////

#include "DXLProServo.h"

/*
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
*/

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

void DXLServo::enableTorque() {			// Enable Dynamixel Torque
    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf( "%s\n", this->pktHandler->getTxRxResult( dxl_comm_result ) );
    }
    else if (dxl_error != 0)
    {
        printf( "%s\n", this->pktHandler->getRxPacketError( dxl_error ) );
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", identity);
    }
}

void DXLServo::disableTorque() {			// Disable Dynamixel Torque
    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
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
        dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_GOAL_POSITION, goalPositionVector[vectorPosition], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }
    }
    //return;
}

void DXLServo::writeGoalPosition(int select, int vectorPosition) {
    if ( select == 0 ) {			// Internal Position Vector
        if( vectorPosition > goalPositionVector.size() || vectorPosition < 0){
            printf("Error! Invalid vector reference!");
            return;
        }
        dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_GOAL_POSITION, goalPositionVector[vectorPosition], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }
    }
    else if ( select == 1 ) {		// Direct entry
        if ( vectorPosition >= 0 && vectorPosition < 4096) {
            dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_GOAL_POSITION, vectorPosition, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
            }
        }
        else {
            printf("Error! Invalid value of desired Goal Position; Values between 0 - 4095 only, from -180 to +180 degrees in units of 0.088 degrees!\n");
        }
    }
    else {
        printf("Error! Invalid Select value; 0 for internal reference or 1 for direct entry!\n");
    }
}

int DXLServo::readCurrentPosition() {
    int position = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, ADDR_MX_PRESENT_POSITION, (uint32_t*)&position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf( "%s\n", this->pktHandler->getTxRxResult( dxl_comm_result ) );
    }
    else if (dxl_error != 0)
    {
        printf( "%s\n", this->pktHandler->getRxPacketError( dxl_error ) );
    }

    if(position < 0){
        printf("Error! Position value is negative! Unexpected Error\n");
        return -1;
    }

    return present_position = position;
}

int DXLServo::checkOperating() {
    uint8_t *set_operate_mode = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, ADDR_MX_OPERATING_MODE, set_operate_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int opMode = 0;
    opMode = (opMode << 8) + set_operate_mode[0];

    if (opMode < 0) {
        printf("Error! Operate Mode value is negative!\n");
        return -1;
    }
    else return opMode;
}

int DXLServo::checkPositionMode() {
    uint8_t *set_operate_mode = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, ADDR_MX_OPERATING_MODE, set_operate_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int opMode = 0;
    opMode = (opMode << 8) + set_operate_mode[0];

    if (opMode < 0) {
        printf("Error! Operate Mode value is negative! Unexpected error!\n");
        return -1;
    }
    else if (opMode != 3) {
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
    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, ADDR_MX_OPERATING_MODE, opMode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }
}

void DXLServo::set4ACurrentLimit() {
    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, ADDR_MX_CURRENT_LIMIT, DXL_CURRENT_LIMIT_4A, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitCurrent = DXL_CURRENT_LIMIT_4A;
}

void DXLServo::setCurrentAmpsLimit(double amps) {
    if (amps > 4.0 || amps < 0) {
        printf("Error! Invalid value of current limit! Select between 0.0 and 4.0!\n");
        return;
    }

    int limit = int( int( (amps / 0.00336) + 0.5 ) );								// Rounding number trick; int(float value + 0.5)

    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, ADDR_MX_CURRENT_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitCurrent = limit;
}

void DXLServo::setCurrentAmpsLimit(int limit) {
    if (limit > 1941 || limit < 0) {
        printf("Error! Invalid value of current limit! Select between 0 and 1941!\n");
        return;
    }
    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, ADDR_MX_CURRENT_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitCurrent = limit;
}

void DXLServo::set80rpmVelLimit() {
    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_VELOCITY_LIMIT, DXL_VELOCITY_LIMIT_80, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitVel = DXL_VELOCITY_LIMIT_80;
}

void DXLServo::setVelocityLimit(int limit) {
    if (limit > 1023 || limit < 0) {
        printf("Error! Invalid value of velocity limit! Select between 0 and 1023!\n");
        return;
    }
    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_VELOCITY_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitVel = limit;
}

int DXLServo::getVelocityLimit(){
    int velocLim = 0;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, ADDR_MX_VELOCITY_LIMIT, (uint32_t*)&velocLim, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    return velocLim;
}

void DXLServo::setlowAccelLimit() {
    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_ACCELERATION_LIMIT, DXL_ACCELERATION_LIMIT_LOW, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitAccel = DXL_ACCELERATION_LIMIT_LOW;
}

void DXLServo::setAccelLimit(int limit) {
    if ( limit > 100 || limit < 0) {
        printf("Error! Invalid value of acceleration limit! Select between 0 and 100!/n100 value limit for safe operation!\n");
        return;
    }
    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_ACCELERATION_LIMIT, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    limitAccel = limit;
}

int DXLServo::getAccelLimit(){
    int accelLim = 0;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, ADDR_MX_ACCELERATION_LIMIT, (uint32_t*)&accelLim, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    return accelLim;
}

void DXLServo::setPositionLimit(bool minMax, int angle) {
	//TOCHANGE: include overload for using position value (0-4095) instead of angle; use double for angle, int for value
    int address = 0;
    if ( !minMax ) {									// minMax == false, Min Position Limit
        address = ADDR_MX_MIN_POSITION_LIMIT;
    }
    else address = ADDR_MX_MAX_POSITION_LIMIT;			// minMax == true, Max Position Limit

    int limit = int( ( double(angle + 180)/0.088 ) + 0.5 );

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
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

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_PROFILE_ACCELERATION, accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    profileAccel = accel;
}

int DXLServo::checkProfileAcceleration() {
    int accel = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, ADDR_MX_PROFILE_ACCELERATION, (uint32_t*)&accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
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

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, ADDR_MX_PROFILE_VELOCITY, vel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    profileVel = vel;
}

int DXLServo::checkProfileVelocity() {
    int vel = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, ADDR_MX_PROFILE_VELOCITY, (uint32_t*)&vel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    if (vel < 0) {
        printf("Error! Value is negative! Unexpected error!\n");
        return -1;
    }
    else return vel;
}

int DXLServo::checkPresentTemperature() {
    uint8_t *limitTemp = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, ADDR_MX_TEMPERATURE_LIMIT, limitTemp, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int limTemp = 0;
    limTemp = (limTemp << 8) + limitTemp[0];

    if (limTemp < 0) {					// Error in comm function return, unexpected error
        printf("Error! Value (EEPROM Limit) is negative! Unexpected error!\n");
        return -1;
    }
    else {									// Return value should be 80 with Temp Limit EEPROM value unchanged
        uint8_t *presTemp = new uint8_t;
        dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, ADDR_MX_PRESENT_TEMPERATURE, presTemp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }

        int valTemp = 0;
        valTemp = (valTemp << 8) + presTemp[0];

        if (valTemp < 0) {
            printf("Error! Value (Presnt Temperature) is negative! Unexpected error!\n");
            return -1;
        }
        else if ( valTemp >= (0.9 * limTemp) && valTemp < limTemp ) {			// If Present Temperature is around 90% of limit or higher but lower than limit
            printf("Warning! Temperature has reached 90 percent of limit!\n");
            return 1;
        }
        else if (valTemp >= limTemp) {					// If Present Temperature is at or exceeds limit ( 80 - 100 C )
            printf("Error! Temperature has exceeded limit! Shutdown triggering!\n");
            return 2;
        }
    }
    return 0;
}

int DXLServo::getPresentTemperature() {
    uint8_t *presTemp = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, ADDR_MX_PRESENT_TEMPERATURE, presTemp, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int valTemp = 0;
    valTemp = (valTemp << 8) + presTemp[0];

    if ( valTemp < 0 || valTemp > 100) {							// Present Temperature valid range: 0 - 100, outside range -> Error
        printf("Error! Value is outside valid range! Unexpected error!\n");
        return -1;
    }
    return valTemp;			// Temperature value ~= Actual Temperature, 1:1 relation
}

int DXLServo::checkPresentCurrent() {
    if (limitCurrent <= 0) {													// Limit should not be 0 or negative
        printf("Error! Limit has not been set or unexpected error has occurred!\n");
        return -1;
    }
    else {
        uint16_t *presCur = new uint16_t;
        dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, ADDR_MX_PRESENT_CURRENT, presCur, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }

        int current = 0;
        current = ( (current << 16) + presCur[0] ) | ( (current << 8) + presCur[1] );

        if (current < 0) {
            printf("Error! Value is negative! Unexpected Error!\n");
            return -1;
        }
        else if (current >= (0.9*limitCurrent) && current < limitCurrent) {
            printf("Warning! Current is close to limit!\n");
            return 1;
        }
        else if (current >= limitCurrent) {
            printf("Error! Current has exceeded limit!\n");
            return 2;
        }
    }
    return 0;
}

double DXLServo::getPresentCurrent() {
    uint16_t *presCur = new uint16_t;
    dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, ADDR_MX_PRESENT_CURRENT, presCur, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int current = 0;
    current = ( (current << 16) + presCur[0] ) | ( (current << 8) + presCur[1] );

    if (current < 0 || current > 1941) {										// Present Current valid range: 0 - 1941
        printf("Error! Value is outside valid range! Unexpected Error!\n");
        return -1;
    }

    double actCurr = static_cast<double>(current);
    actCurr = actCurr * 0.00336;
    //current = current * 0.00336;												// Present Current value * 3.36mA = Actual Current {uncomment if conversion wanted}
    return actCurr;
}

int DXLServo::getHomingOffset(){
    int homeOffset = 0;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, ADDR_MX_HOMING_OFFSET, (uint32_t*)&homeOffset, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    return homeOffset;
}

////////////////////////////////////////////////////   End of DXLServo class   /////////////////////////////////////////////////////////////////////////////////////////////

/*
////////////////////
//		29 Mar 2018 changes not yet implemented in example.
//      21 May: Linux test
//		31 May: Updated working test code to read DXL status and write position to tilt Servo.
////////////////////

////////////////////////////////////////////////////    Example Code Using DXLServo functions    ///////////////////////////////////////////////////////////////////////////
int main() {
    // Initialize Servo objects and parameters

    DXLServo panServo;								// DXLServo object 1
    if ( panServo.getDXLID() != 1 ) {				// Ensure Pan set to ID 1
        panServo.setDXLID(1);
    }
    panServo.setDeviceName(0);						// USB port ttyUSB0
    
    DXLServo tiltServo;								// DXLServo object 2
    if (tiltServo.getDXLID() != 2) {				// Ensure Tilt set to ID 2
        tiltServo.setDXLID(2);
    }
    tiltServo.setDeviceName(0);						// USB port ttyUSB0

    // Begin DXL operations
	// Open Comms ports
    int fd = 0;
    fd = open(panServo.deviceName.c_str(), O_CREAT | O_RDWR | O_NOCTTY | O_NDELAY);
    //fd = open(panServo.deviceName.c_str(), O_RDWR | O_NOCTTY);
    if(fd == -1)    cout << "unable to open USB" << endl;
    else    cout << "USB Port Opened" << endl;

    //init
    panServo.initPortHandler();
    panServo.initPacketHandler();
    panServo.openPort();
    panServo.setPortBaudRate();

    tiltServo.initPortHandler();
    tiltServo.initPacketHandler();
    tiltServo.openPort();
    tiltServo.setPortBaudRate();

    cout << "Ports Opened" << endl;

    //EEPROM Write, do before enabling Torque
    //tiltServo.setOperatingMode(2);  //Set operating mode to Position Mode, internal = 3
    tiltServo.set4ACurrentLimit();  //Set Current Limit to 4A, val = 1190
    tiltServo.set80rpmVelLimit();
    int velLimit = tiltServo.getVelocityLimit();
    cout << "Velocity Limit: " << velLimit << " rpm." << endl;
    tiltServo.setlowAccelLimit();
    int accLimit = tiltServo.getAccelLimit();
    cout << "Accel Limit: " << accLimit << " rev/min2." << endl;
    //tiltServo.setCurrentAmpsLimit(3.5);    //Set current limit to 3.5A, val = 1042
    //tiltServo.setCurrentAmpsLimit(1200);    //Set current Limit to val = 1200, 4.032A

    // Enable Torque outputs
    panServo.enableTorque();
    tiltServo.enableTorque();
    cout << "Torque Enabled" << endl;

    //int pan = 0, tilt = 0;
    int currentPanPos = 0, currentTiltPos = 0, currentPanTemp = 0, currentTiltTemp = 0, homePanOffset = 0, homeTiltOffset = 0;
    double currentPanI = 0, currentTiltI = 0;
    int cycle = 3;

    while (cycle > 0) {
        sleep(2);	// 2 sec delay

        // Read current positions        
        currentPanPos = panServo.readCurrentPosition();
        homePanOffset = panServo.getHomingOffset();
        cout << "Pan Offset: " << homePanOffset << "." << endl;
        double anglePan = ( (double)currentPanPos * 0.088 ) - 180 - (homePanOffset * 0.088);
        cout << "Pan Position: " << anglePan << " degrees." << endl;

        currentTiltPos = tiltServo.readCurrentPosition();
        homeTiltOffset = tiltServo.getHomingOffset();
        cout << "Tilt Offset: " << homeTiltOffset << "." << endl;
        cout << "Tilt Pos Val: " << currentTiltPos << "." << endl;
        double angleTilt = ( (double)currentTiltPos * 0.088 ) - 180 - (homeTiltOffset * 0.088);
        cout << "Tilt Position: " << angleTilt << " degrees." << endl;

        currentPanTemp = panServo.getPresentTemperature();
        cout << "Pan Temp: " << currentPanTemp << "C." << endl;
        currentTiltTemp = tiltServo.getPresentTemperature();
        cout << "Tilt Temp: " << currentTiltTemp << "C." << endl;

        currentPanI = panServo.getPresentCurrent();
        cout << "Pan Current: " << std::fixed << std::setprecision(2) << currentPanI << "A." << endl;
        currentTiltI = tiltServo.getPresentCurrent();
        cout << "Tilt Current: " << std::fixed << std::setprecision(2)  << currentTiltI << "A." << endl;

        tiltServo.setProfileAcceleration(5);
        int profileAccel = tiltServo.checkProfileAcceleration();
        cout << "Profile Acc Val: " << profileAccel << " ." << endl;
        tiltServo.setProfileVelocity(10);
        int profileVel = tiltServo.checkProfileVelocity();
        cout << "Profile Vel Val: " << profileVel << " ." << endl;

        tiltServo.writeGoalPosition(1);
        std::vector<int> tiltGoals;
        tiltGoals.clear();
        tiltGoals.push_back(1830);
        tiltGoals.push_back(1900);
        tiltGoals.push_back(1700);
        tiltServo.addToGoalPosVector(tiltGoals);
        int checkGPV = tiltServo.checkGoalPosVector();
        tiltServo.writeGoalPosition(1);
        sleep(2);
        tiltServo.writeGoalPosition(1, 2000);
        sleep(2);
        tiltServo.writeGoalPosition(0, 2);
        sleep(2);

        cycle -= 1;
    }

    // Disable DXLs Torque outputs
    panServo.disableTorque();
    tiltServo.disableTorque();
    cout << "Torque Disabled" << endl;

    //panServo.prtHandler->closePort();
    close(fd);
    // Close Comms port
    panServo.closePort();
    tiltServo.closePort();
    cout << "Ports Closed" << endl;

    return 0;
}
// End of C++ Example Code using DXLServo functions
*/

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
