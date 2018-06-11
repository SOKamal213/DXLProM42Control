
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
Date: 8 June 2018

For Dynamixel MX-64 and Pro M42 servos specifically. Control tables for MX and Pro servos in header file. Code allows use of either servo by setting servoType variable when creating DXLServo object.
Most Register addresses added as required by project. Unused ones may be added in future. 
Most basic functionalities added; 
- Position control via direct integer value or angle. 
- Velocity and Acceleration changes via direct integer or velocity/acceleration. 
- Current or Torque (depending on servo type) changes via direct integer or current. 
- Read/Write of most used Parameters (Position, Velocity, Acceleration, Current/Torque, Present values, Limits, Gains, internal states, etc). 
- Conversion between high level (angle, velocity, acceleration) and low level integers

Code built in Linux Ubuntu environment. Cross platform functionality not fully implemented yet. Focus on Ubuntu for now.

Requires Dynamixel SDK:
https://github.com/ROBOTIS-GIT/DynamixelSDK
eManual for environment Setup
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

///////////////////////////////////////////////////////////////////////////////*/

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

void crossSleep(int time) {			// Cross platform sleep() func. Linux/Unix: time in seconds. Windows: time in milliseconds. May be busy sleep. Change to non-busy sleep.
#if defined(__linux__) || defined(__APPLE__)
    if (time >= 10) {
        printf("Sleeping for more than 10 seconds. System will not respond in that time. Are you sure?\n");
    }
    sleep(time);		// Change to ms sleep later
#elif defined(_WIN32) || defined(_WIN64)
    _sleep(time);
#endif
}

////////////////////////////////////////////////////   DXLServo class definition   /////////////////////////////////////////////////////////////////////////////////////////

DXLServo::DXLServo() {							// Constructor
    identity = 1;								// default ID 1
    protocolVersion = 2.0;						// default Protocol 2.0
    baudRate = 57600;							// default 57,600 baud

    goalPositionVector.clear();					// Initialize empty vectors at start
	goalAngleVector.clear();
    present_position = 0, present_current = 0.0, present_temperature = 25;				// Basic starting values, change with read later
    limitAccel = 1, limitVel = 1, limitCurrent = 0, limitPosMin = 0, limitPosMax = 4095, homeOffset = 0;
    profileAccel = 1, profileVel = 1;

    for (int i = 0; i < 4; i++) {			// Initialize all External Port Modes to 0;
        externalPort[i] = 0;
    }


    dxl_comm_result = COMM_TX_FAIL;				// Initial comm result
    dxl_error = 0;								// Dynamixel error
}

DXLServo::~DXLServo() {							// Destructor

}

void DXLServo::enableTorque() {			// Enable Dynamixel Torque
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_TORQUE_ENABLE;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_TORQUE_ENABLE;

    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, address, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", identity);
    }
}

void DXLServo::disableTorque() {			// Disable Dynamixel Torque
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_TORQUE_ENABLE;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_TORQUE_ENABLE;

    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, address, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }
}

void DXLServo::addToGoalPosVector(const std::vector<int> &posVector) {			// Store a vector of preselected Goal Positions into internal vector
    for (int i = 0; i < posVector.size(); i++) {
        goalPositionVector.push_back(posVector[i]);
    }
}

int DXLServo::checkGoalPosVector() {			// Check internal vector for invalid data
    if (goalPositionVector.size() == 0) {
        printf("Error! Goal Position Vector is empty!\n");
        return -1;
    }
    else {
        bool check = false;
        int limit;
        if (servoType == DXL_MX_64) {
            limit = 4095;
        }
        else if (servoType == DXL_PRO_M42) {
            limit = 131593;
        }
        for (int i = 0; i < goalPositionVector.size(); i++) {
            if (abs(goalPositionVector[i]) > limit) {
                printf("Error! Goal Position Vector contains invalid position at index %d!\n", i);
                printf("!!! Modify or reinitialize Position Vector !!!\n");
                check = true;
                break;			// Exit for loop at any point error occurs
            }
            // if (servoType == DXL_MX_64 && goalPositionVector[i] < 0) {
                // printf("Error! MX servo cannot have negative position at index %d!\n", i);
                // printf("!!! Modify or reinitialize Position Vector !!!\n");
                // check = true;
                // break;			// Exit for loop at any point error occurs
            // }
        }
        if (check) return 0;
        else return 1;
    }
}

void DXLServo::writeGoalPosition(int vectorPosition) {						// Write DXL position from internal stored vector. Input: vectorPosition = index of positon stored in internal vector
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_GOAL_POSITION;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_GOAL_POSITION;

    if (goalPositionVector.size() == 0) {				// If array is empty
        printf("Error! Goal Position Vector is empty!\n");
    }
    else {
        dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, goalPositionVector[vectorPosition], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }
        int count = 0;
        while (isMoving()) {		// internal Moving check to disallow actions while not in position. Comment out if changing to external Moving check
            count += 1;
            if( (count % 30) == 0 )    printf("Servo still moving...\n");
            //crossSleep(1);		// Assume Linux, sleep for 1s
            //sleep(1);			// Unix, Linux: sleep for 1s
            //_sleep(500);		// Windows: sleep for 0.5s
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));	// C++ standard library non-busy sleep
        }
        count = 0;
    }
    //return;
}

void DXLServo::writeGoalPosition(int select, int vectorPosition) {			// Write DXL position from either internal Position or direct integer entry. Inputs: select: 0 = Internal vector, 1 = direct entry. vectorPosition: Index of stored position in internal vector or direct integer value of desired position
    int address, limit;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_GOAL_POSITION;
        limit = 4096;		// Max integer value for position of MX servo
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_GOAL_POSITION;
        limit = pow(2, 31);		// Max integer value for position of Pro servo
    }
    if (select == 0) {			// Internal Position Vector
        if (vectorPosition > goalPositionVector.size() || vectorPosition < 0) {
            printf("Error! Invalid vector reference!");
            return;
        }
        dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, goalPositionVector[vectorPosition], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }
    }
    else if (select == 1) {		// Direct entry
        if (abs(vectorPosition) >= 0 && abs(vectorPosition) < limit) {
            dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, vectorPosition, &dxl_error);
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
            printf("Error! Invalid value of desired Goal Position; Values between 0 - %i only, from -180 to +180 degrees in units of 0.088 degrees!\n", limit);
        }
    }
    else {
        printf("Error! Invalid Select value; 0 for internal reference or 1 for direct entry!\n");
    }
    int count = 0;
    while (isMoving()) {		// internal Moving check to disallow actions while not in position. Comment out if changing to external Moving check
        count += 1;
        if( (count % 30) == 0 )    printf("Servo still moving...\n");
        //crossSleep(1);		// Assume Linux, sleep for 1s
        //sleep(1);			// Unix, Linux: sleep for 1s
        //_sleep(500);		// Windows: sleep for 0.5s
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));	// C++ standard library non-busy sleep
    }
    count = 0;
}

int DXLServo::readCurrentPosition() {			// Read servo's position, returns integer value. For angle, use readCurrentAngle().
    bool success = true;
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PRESENT_POSITION;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_PRESENT_POSITION;

    printf("Reading Present Position\n");
    int position = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Present Position read\n");
        if (position < 0) {
            printf("Error! Position value is negative! Unexpected Error\n");
            return -1;
        }
        present_position = position;
    }
    return position;
}

double DXLServo::readCurrentAngle() {			// Read servo's position, returns in angle (degrees).
    bool success = true;
    int address, temp;
    double angle;

    if (servoType == DXL_MX_64)  address = ADDR_MX_PRESENT_POSITION;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_PRESENT_POSITION;

    printf("Reading Present Angle\n");
    int position = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Present Angle read\n");
//        if (position < 0 && servoType == DXL_MX_64) {
//            printf("Error! Position value is negative! Unexpected Error\n");
//            return -1.0;
//        }
//        else if (position < 0 && servoType == DXL_PRO_M42) {
//            temp = -(position);
//        }
//        else{
        temp = position;
        //}
        angle = convertValtoPos( (temp + homeOffset) );		// Current angle including homing offset
        present_position = temp;
    }
    return angle;
}

void DXLServo::addToGoalAngleVect(const std::vector<double> &angVector) {		// Store a vector of preselected Goal Positions into internal vector
    for (int i = 0; i < angVector.size(); i++) {
        goalAngleVector.push_back(angVector[i]);
    }
}

int DXLServo::checkGoalAngleVect() {			// Check internal vector for invalid data
    if (goalAngleVector.size() == 0) {
        printf("Error! Goal Angle Vector is empty!\n");
        return -1;
    }
    else {
        bool check = false;
        double limit = 180.0;
//        if (servoType == DXL_MX_64) {
//            limit = 360;
//        }
//        else if (servoType == DXL_PRO_M42) {
//            limit = 180;
//        }
        for (int i = 0; i < goalAngleVector.size(); i++) {
            if (abs(goalAngleVector[i]) > limit) {
                printf("Error! Exceeded limit at index %d!\n", i);
                printf("!!! Modify or reinitialize Position Vector !!!\n");
                check = true;
                break;			// Exit for loop at any point error occurs
            }
//            if (servoType == DXL_MX_64 && goalAngleVector[i] < 0) {
//                printf("Error! MX servo cannot have negative position at index %d!\n", i);
//                printf("!!! Modify or reinitialize Position Vector !!!\n");
//                check = true;
//                break;			// Exit for loop at any point error occurs
//            }
        }
        if (check) return 0;
        else return 1;
    }
}

void DXLServo::writeGoalAngle(int vectorAngle) {			// Write DXL position from internal stored vector. Input: vectorAngle = index of positon stored in internal vector
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_GOAL_POSITION;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_GOAL_POSITION;

    if (goalAngleVector.size() == 0) {				// If array is empty
        printf("Error! Goal Position Vector is empty!\n");
        return;
    }
    else if (vectorAngle > goalAngleVector.size() || vectorAngle < 0) {		// if reference to vector is negative or exceeds vector size
        printf("Error! Invalid vector reference!");
        return;
    }
    else {
        int valPos = convertPostoVal(goalAngleVector[vectorAngle]);
        valPos = valPos - homeOffset;
        dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, valPos, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }
        int count = 0;
        while (isMoving()) {			// internal Moving check to disallow actions while not in position. Comment out if changing to external Moving check
            count += 1;
            if( (count % 30) == 0 )    printf("Servo still moving...\n");
            //crossSleep(1);		// Assume Linux, sleep for 1s
            //sleep(1);			// Unix, Linux: sleep for 1s
            //_sleep(500);		// Windows: sleep for 0.5s
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));	// C++ standard library non-busy sleep
        }
        count = 0;
        //printf("Goal angle write done\n");
    }
}

void DXLServo::writeGoalAngle(int select, double vectorAngle) {			// Write DXL position from either internal Angle or direct integer entry. Inputs: select: 0 = Internal vector, 1 = direct entry. vectorPosition: Index of stored position in internal vector or direct integer value of desired position
    int address, limit;
    double unit;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_GOAL_POSITION;
        limit = 4096;
        unit = 0.088;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_GOAL_POSITION;
        limit = pow(2, 31);
        unit = 0.00137;
    }

    if (select == 0) {			// Internal Position Vector
        if (vectorAngle > goalAngleVector.size() || vectorAngle < 0) {
            printf("Error! Invalid vector reference!");
            return;
        }
        int valPos = convertPostoVal(goalAngleVector[vectorAngle]);
        valPos = valPos - homeOffset;
        dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, valPos, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }
    }
    else if (select == 1) {		// Direct entry
        int valPos = convertPostoVal(vectorAngle);
        valPos = valPos - homeOffset;
        if (abs(valPos) < limit) {
            dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, valPos, &dxl_error);
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
            printf("Error! Invalid value of desired Goal Position; Values between 0 - %i only, from 0 to 360 degrees in units of %f degrees!\n", limit, unit);
        }
    }
    else {
        printf("Error! Invalid Select value; 0 for internal reference or 1 for direct entry!\n");
    }
    int count = 0;
    while (isMoving()) {			// internal Moving check to disallow actions while not in position. Comment out if changing to external Moving check
        count += 1;
        if( (count % 30) == 0 )    printf("Servo still moving...\n");
        //crossSleep(1);		// Assume Linux, sleep for 1s
        //sleep(1);			// Unix, Linux: sleep for 1s
        //_sleep(500);		// Windows: sleep for 0.5s
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));	// C++ standard library non-busy sleep
    }
    count = 0;
}

int DXLServo::checkOperating() {			// Returns status of Operate Mode register
    //bool success = true;
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_OPERATING_MODE;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_OPERATING_MODE;

    printf("Reading Operate Mode\n");
    uint8_t *set_operate_mode = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, address, set_operate_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        //success = false;
    }

    printf("Operate Mode read\n");
    int opMode = 0;
    opMode = (opMode << 8) + set_operate_mode[0];

    if (opMode < 0) {
        printf("Error! Operate Mode value is negative!\n");
        return -1;
    }
    else return opMode;
}

int DXLServo::checkPositionMode() {				// Checks if Operate Mode is set to Position Control Mode, warning if not.
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_OPERATING_MODE;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_OPERATING_MODE;

    uint8_t *set_operate_mode = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, address, set_operate_mode, &dxl_error);
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

void DXLServo::setOperatingMode(int mode) {				// Change Operate Mode. Input: mode: 0 = Current Control, 1 = Velocity Control, 2 = Position (Single turn) Control (default), 3 = Extended Position (Multi turn) Control, 4 = Current based Position Control, 5 = PWM Control
    bool success = true;
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_OPERATING_MODE;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_OPERATING_MODE;

    uint8_t opMode = 3;									// Default Operating Mode: 3, Position Control
    std::string modeSet;
    // add MX or PRO check
    switch (mode) {
    case 0:
        opMode = DXL_MX_CURRENT_CONTROL_MODE;
        modeSet = "Current";
        break;
    case 1:
        opMode = DXL_VELOCITY_CONTROL_MODE;
        modeSet = "Velocity";
        break;
    case 2:
        opMode = DXL_POSITION_CONTROL_MODE;
        modeSet = "Position";
        break;
    case 3:
        opMode = DXL_EXTENDED_CONTROL_MODE;
        modeSet = "Extended Position";
        break;
    case 4:
        opMode = DXL_MX_CURRENT_POSITION_CONTROL_MODE;
        modeSet = "Current Position";
        break;
    case 5:
        opMode = DXL_MX_PWM_CONTROL_MODE;
        modeSet = "PWM";
        break;
    }

    if (servoType == DXL_PRO_M42 && (opMode == 4 || opMode == 5)) {
        printf("ERROR! Pro servo does not implement Current based Position or PWM Control!\n");
        return;
    }

    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, address, opMode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //printf("Operating Mode set to: %s Control Mode\n", modeSet);
        cout << "Operating Mode set to: " << modeSet << " Control Mode." << endl;
    }
}

void DXLServo::setMaxCurrentLimit() {       // Sets Current limit to the max value before stall torque occurs
    bool success = true;
    int address, limit;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_CURRENT_LIMIT;
        limit = DXL_MX_CURRENT_LIMIT_MAX;
        //mult = 0.00336;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_TORQUE_LIMIT;
        limit = DXL_PRO_M42_TORQUE_LIMIT_MAX;
        //mult = 0.004028;
    }

    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //double set = limit * mult;
        double set = convertValtoCurr(limit);
        printf("Current limit set to max %.2f amps\n", set);
        limitCurrent = limit;
    }
}

void DXLServo::setCurrentLimit(double amps) {		// Sets Current limit to desired current value. Limited by precalculated max current before stall torque occurs.
    bool success = true;
    int address;
    double currLimit;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_CURRENT_LIMIT;
        currLimit = 4.0;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_TORQUE_LIMIT;
        currLimit = 2.1;
    }

    if (amps > currLimit || amps < 0.0) {
        printf("Error! Invalid value of current limit! Select between 0.0 and %f!\n", currLimit);
        return;
    }

    if (amps < (0.2 * currLimit)) {
        printf("Warning! Setting value to less than 0.2 of limit, \nServo may not function properly under load!\n");
    }

    int limit;
    limit = convertCurrtoVal(amps);
    //if (servoType == DXL_MX_64) {
    //	limit = int(int((amps / 0.00336) + 0.5));								// Rounding number trick; int(float value + 0.5)
    //}
    //else {
    //	limit = int(int((amps / 0.004028) + 0.5));
    //}

    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        //success = false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Current Limit set to %.2f amps\n", amps);
        limitCurrent = limit;
    }
}

void DXLServo::setCurrentLimit(int limit) {				// Sets Current limit via direct integer value. Limited by MX value limit or Pro M42 servo corresponding max current before stall torque occurs
    bool success = true;
    int address, currLimit;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_CURRENT_LIMIT;
        currLimit = 1941;
        //mult = 0.00336;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_TORQUE_LIMIT;
        currLimit = 521;
        //mult = 0.004028;
    }

    if (limit > currLimit || limit < 0) {
        printf("Error! Invalid value of current limit! Select between 0 and %d!\n", currLimit);
        return;
    }

    if (limit < (currLimit / 5)) {
        printf("Warning! Setting value to less than 0.2 of limit, \nservo may not function properly under load!\n");
    }

    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //double set = double(limit)* mult;
        double set = convertValtoCurr(limit);
        printf("Current Limit set to %.2f\n", set);
        limitCurrent = limit;
    }
}

void DXLServo::set80rpmVelLimit() {			// Sets Velocity Limit to 80rpm
    bool success = true;
    int address, velLimit;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_VELOCITY_LIMIT;
        velLimit = DXL_MX_VELOCITY_LIMIT_80;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_VELOCITY_LIMIT;
        velLimit = DXL_PRO_M42_VELOCITY_LIMIT_80;
    }

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, velLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = true;
    }

    if (success) {
        printf("Velocity Limit set to 80rpm\n");
        limitVel = velLimit;
    }
}

void DXLServo::setVelocityLimit(int limit) {			// Sets Velocity limit via direct integer value
    bool success = true;
    int address, velLimit;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_VELOCITY_LIMIT;
        velLimit = 1023;
        //mult = 0.229;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_VELOCITY_LIMIT;
        velLimit = 25710;       // Actual limit for Pro servo is 2^31, but unlikely to use such high speed, severely limited for position application
        //mult = 0.00389076;
    }

    if (limit > velLimit || limit < 0) {
        printf("Error! Invalid value of velocity limit! Select between 0 and %d!\n", velLimit);
        return;
    }
    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //int set = int(limit * mult);
        int set = convertValtoVel(limit);
        printf("Velocity Limit set to %i rpm\n", set);
        limitVel = limit;
    }
}

void DXLServo::setVelocityLimit(double limit) {			// Set velocity limit to desired velocity (rpm)
    bool success = true;
    int address;
    double velLimit = 235.0;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_VELOCITY_LIMIT;
        //velLimit = 1023;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_VELOCITY_LIMIT;
        //velLimit = 60400;       // Actual limit for Pro servo is 2^31 * 0.00389076, but unlikely to use such high speed, severely limited for position application
    }

    if (limit > velLimit || limit < 0) {
        printf("Error! Invalid value of velocity limit! Select between 0 and 235.0!\n");
        return;
    }

    int pass = convertVeltoVal(limit);

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, pass, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Velocity Limit set to %.3f rpm\n", limit);
        limitVel = pass;
    }
}

int DXLServo::getVelocityLimit() {				// Read value of Velocity Limit set in servo. Returns integer value, conversion available. Change func to return double instead of int?
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_VELOCITY_LIMIT;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_VELOCITY_LIMIT;

    int velocLim = 0;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&velocLim, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    double velocity = convertValtoVel(velocLim);
    printf("Velocity Limit at %f rpm\n", velocity);
    //return velocity;
    return velocLim;
}

void DXLServo::setlowAccelLimit() {				// Sets Acceleration to low precalculated value. DXL servo acceleration far too high for position application, causes following error and shutdown.
    bool success = true;
    int address, limit;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_ACCELERATION_LIMIT;
        limit = DXL_MX_ACCELERATION_LIMIT_LOW;
        //mult = 214.577;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_ACCELERATION_LIMIT;
        limit = DXL_PRO_M42_ACCEL_LIMIT_LOW;
        //mult = 201.039;
    }

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //double set = limit * mult;
        double set = convertValtoAcc(limit);
        printf("Acceleration set to %.3f rev/min2\n", set);
        limitAccel = DXL_MX_ACCELERATION_LIMIT_LOW;
    }
}

void DXLServo::setAccelLimit(int limit) {				// Sets Acceleration limit, limited to 100 * unit. Modifiable.
    bool success = true;
    int address;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_ACCELERATION_LIMIT;
        //mult = 214.577;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_ACCELERATION_LIMIT;
        //mult = 201.039;
    }

    if (limit > 100 || limit < 0) {
        printf("Error! Invalid value of acceleration limit! Select between 0 and 100!/n100 value limit for safe operation!\n");
        return;
    }
    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //double set = limit * mult;
        double set = convertValtoAcc(limit);
        printf("Acceleration set to %.3f rev/min2\n", set);
        limitAccel = limit;
    }
}

void DXLServo::setAccelLimit(double limit) {			// Sets Acceleration limit, limited to 100 * unit.
    bool success = true;
    int address;
    double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_ACCELERATION_LIMIT;
        mult = 214.577;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_ACCELERATION_LIMIT;
        mult = 201.039;
    }

    if (limit > (100 * mult) || limit < 0) {
        printf("Error! Invalid value of acceleration limit! Select between 0 and 100 * %.3f!/n100 value limit for safe operation!\n", mult);
        return;
    }

    int pass = convertAcctoVal(limit);

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, pass, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Acceleration set to %.3f rev/min2\n", limit);
        limitAccel = pass;
    }
}

int DXLServo::getAccelLimit() {				// Read value of Acceleration limit, return in integer. Conversion available.
    bool success = true;
    int address;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_ACCELERATION_LIMIT;
        //mult = 214.577;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_ACCELERATION_LIMIT;
        //mult = 201.039;
    }

    printf("Reading acceleration limit\n");

    int accelLim = 0;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&accelLim, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) printf("Acceleration limit read\n");
    //double accel = accelLim * mult;
    double accel = convertValtoAcc(accelLim);
    printf("Acceleration Limit at  %.3f rev/min2\n", accel);
    return accelLim;
}

void DXLServo::setPositionLimit(bool minMax, int position) {		// Sets Position limits of servo. Inputs: minMax false for Min, true for Max; Position int for direct transmission
    bool success = true;
    int address, limitMin, limitMax;
    std::string set;
    if (!minMax) {									// minMax == false, Min Position Limit
        if (servoType == DXL_MX_64) {
            address = ADDR_MX_MIN_POSITION_LIMIT;
            limitMin = 0;
        }
        else if (servoType == DXL_PRO_M42) {
            address = ADDR_PRO_MIN_POSITION_LIMIT;
            limitMin = int( -(pow(2.0,31.0)) );
        }
        set = "Minimum";
    }
    else {
        if (servoType == DXL_MX_64) {
            address = ADDR_MX_MAX_POSITION_LIMIT;			// minMax == true, Max Position Limit
            limitMax = 4096;
        }
        else if (servoType == DXL_PRO_M42) {
            address = ADDR_PRO_MAX_POSITION_LIMIT;
            limitMax = int( pow(2.0, 31.0) );
        }
        set = "Maximum";
    }

    if (position < limitMin || position > limitMax) {		// check that position does not exceed limits
        printf("ERROR! Value exceeds limits!\nSelect between %i and %i!\n", limitMin, limitMax);
        return;
    }

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        double angle = convertValtoPos(position);
        cout << set << " Position limit set to " << angle << " degrees" << endl;
        if (!minMax)	limitPosMin = position;
        else			limitPosMax = position;
    }
}

void DXLServo::setPositionLimit(bool minMax, double angle) {		// Sets Position limits of servo. Inputs: minMax false for Min, true for Max; angle in degrees
    bool success = true;
    int address;
    double limitMin, limitMax;
    std::string set;
    if (!minMax) {									// minMax == false, Min Position Limit
        if (servoType == DXL_MX_64) {
            address = ADDR_MX_MIN_POSITION_LIMIT;
            limitMin = 0.0;
        }
        else if (servoType == DXL_PRO_M42) {
            address = ADDR_PRO_MIN_POSITION_LIMIT;
            limitMin = -( pow(2.0, 31.0) );
        }
        set = "Minimum";
    }
    else {
        if (servoType == DXL_MX_64) {
            address = ADDR_MX_MAX_POSITION_LIMIT;			// minMax == true, Max Position Limit
            limitMax = 360.0;
        }
        else if (servoType == DXL_PRO_M42) {
            address = ADDR_PRO_MAX_POSITION_LIMIT;
            limitMax = int( pow(2.0, 31.0) );
        }
        set = "Maximum";
    }

    if (angle < limitMin || angle > limitMax) {		// check that angle does not exceed limits
        printf("ERROR! Value exceeds limits!\nSelect between %f and %f!\n", limitMin, limitMax);
        return;
    }

    int position = convertPostoVal(angle);

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        cout << set << " Position limit set to " << angle << " degrees" << endl;
        if (!minMax)	limitPosMin = position;
        else			limitPosMax = position;
    }
}

void DXLServo::setProfileAcceleration(int accel) {			// Sets Acceleration value for move while servo torque is enabled. Input: accel: direct integer value for transmission.
    bool success = true;
    int address;
    //double mult;
    if (servoType == DXL_MX_64){
        address = ADDR_MX_PROFILE_ACCELERATION;
        //mult = 214.577;
    }
    else if (servoType == DXL_PRO_M42) {
        //// Can edit to use Goal Var if Pro servo used to distinguish between speed control and Positioning Move Profile.
        //printf("Error! Profile Variable only for MX servos!\nUse Goal Acceleration for Pro servo!\n");
        //return;
        address = ADDR_PRO_GOAL_ACCELERATION;
        //mult = 201.039;
    }

    if (accel == 0) {
        printf("Error! Value of 0 will give infinite acceleration! Disallowed for safe operation!\n");
        return;
    }
    // Assumes Acceleration limit has been set/read in code before
    else if (accel > limitAccel || accel < 0) {
        printf("Error! Invalid value! Select value between 1 - %d!\n", limitAccel);
        return;
    }

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        //double set = double(accel) * 214.577;
        double set = convertValtoAcc(accel);
        printf("Profile acceleration set to %.3f rev/min2\n", set);
        profileAccel = accel;
    }
}

void DXLServo::setProfileAcceleration(double accel) {			// Sets Acceleration value for move while servo torque is enabled. Input: accel: acceleration (rpm2)
    bool success = true;
    int address;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_PROFILE_ACCELERATION;
        //mult = 214.577;
    }
    else if (servoType == DXL_PRO_M42) {
        //// Can edit to use Goal Var if Pro servo used to distinguish between speed control and Positioning Move Profile.
        //printf("Error! Profile Variable only for MX servos!\nUse Goal Acceleration for Pro servo!\n");
        //return;
        address = ADDR_PRO_GOAL_ACCELERATION;
        //mult = 201.039;
    }
    // Assumes Acceleration limit has been set/read in code before
    double limit = convertValtoAcc(limitAccel);

    if (accel == 0) {
        printf("Error! Value of 0 will give infinite acceleration! Disallowed for safe operation!\n");
        return;
    }
    else if (accel > limit || accel < 0) {
        printf("Error! Invalid value! Select value between 1 - %f!\n", limit);
        return;
    }

    // Convert to integer for transmission
    int pass = convertAcctoVal(accel);

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Profile acceleration set to %.3f rev/min2\n", accel);
        profileAccel = pass;
    }
}

int DXLServo::checkProfileAcceleration() {				// Read value of Profile Acceleration register, returns direct integer value. Conversion available.
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PROFILE_ACCELERATION;
    else if (servoType == DXL_PRO_M42) {
        //printf("Error! Profile Variable only for MX servos!\nUse Goal Acceleration for Pro servo!\n");
        //return -1;
        address = ADDR_PRO_GOAL_ACCELERATION;
    }
    printf("Reading Profile Acceleration\n");
    int accel = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&accel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    printf("Profile Acceleration read\n");
    double trueAccel = convertValtoAcc(accel);

    if (accel < 0) {
        printf("Error! Value is negative! Unexpected error!\n");
        return -1;
    }
    else return accel;
}

void DXLServo::setProfileVelocity(int vel) {			// Sets Velocity value for move while servo torque is enabled. Input: vel: direct integer value for transmission
    bool success = true;
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PROFILE_VELOCITY;
    else if (servoType == DXL_PRO_M42) {
        //printf("Error! Profile Variable only for MX servos!\nUse Goal Velocity for Pro servo!\n");
        //return;
        address = ADDR_PRO_GOAL_VELOCITY;
    }

    if (vel == 0) {
        printf("Warning! Value of 0 will give infinite velocity! Disallowed for safe operation!\n");
        return;
    }
    // Assumes Velocity limit has been set/read in code before
    else if (vel > limitVel || vel < 0) {
        printf("Error! Invalid value! Select value between 1 - %d!\n", limitVel);
        return;
    }

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, vel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        double set = convertValtoVel(vel);
        printf("Profile Velocity set to %.3f rpm\n", set);
        profileVel = vel;
    }
}

void DXLServo::setProfileVelocity(double vel) {			// Sets Velocity value for move while servo torque is enabled. Input: vel: velocity (rpm)
    bool success = true;
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PROFILE_VELOCITY;
    else if (servoType == DXL_PRO_M42) {
        //printf("Error! Profile Variable only for MX servos!\nUse Goal Velocity for Pro servo!\n");
        //return;
        address = ADDR_PRO_GOAL_VELOCITY;
    }

    double limit = convertValtoVel(limitVel);

    if (vel == 0) {
        printf("Warning! Value of 0 will give infinite velocity! Disallowed for safe operation!\n");
        return;
    }
    // Assumes Velocity limit has been set/read in code before
    else if (vel > limit || vel < 0) {
        printf("Error! Invalid value! Select value between 1 - %d!\n", limitVel);
        return;
    }

    int pass = convertVeltoVal(vel);

    dxl_comm_result = this->pktHandler->write4ByteTxRx(this->prtHandler, identity, address, pass, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Profile Velocity set to %.3f rpm\n", vel);
        profileVel = pass;
    }
}

int DXLServo::checkProfileVelocity() {				// Reads value of register in servo, returns direct integer value. Conversion available.
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PROFILE_VELOCITY;
    else if (servoType == DXL_PRO_M42) {
        //printf("Error! Profile Variable only for MX servos!\nUse Goal Velocity for Pro servo!\n");
        //return -1;
        address = ADDR_PRO_GOAL_VELOCITY;
    }

    int vel = -1;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&vel, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    double trueVel = convertValtoVel(vel);		// convert to rpm, for future use

    if (vel < 0) {
        printf("Error! Value is negative! Unexpected error!\n");
        return -1;
    }
    else return vel;
}

int DXLServo::checkPresentTemperature() {			// Checks temperature of servo, issues warnings for close to limit and exceeded limit. ADD: shutdown check if exceed.
    int addressLim, addressTemp;
    if (servoType == DXL_MX_64) {
        addressLim = ADDR_MX_TEMPERATURE_LIMIT;
        addressTemp = ADDR_MX_PRESENT_TEMPERATURE;
    }
    else if (servoType == DXL_PRO_M42) {
        addressLim = ADDR_PRO_TEMPERATURE_LIMIT;
        addressTemp = ADDR_PRO_PRESENT_TEMPERATURE;
    }

    uint8_t *limitTemp = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, addressLim, limitTemp, &dxl_error);
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
        dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, addressTemp, presTemp, &dxl_error);
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
        else if (valTemp >= (0.9 * limTemp) && valTemp < limTemp) {			// If Present Temperature is around 90% of limit or higher but lower than limit
            printf("Warning! Temperature has reached 90 percent of limit!\n");
            return 1;
        }
        else if (valTemp >= limTemp) {					// If Present Temperature is at or exceeds limit ( 80 - 100 C )
            printf("Error! Temperature has exceeded limit! Shutdown triggering!\n");
			//ADD checkShutdown()
            return 2;
        }
    }
    return 0;
}

int DXLServo::getPresentTemperature() {				// Reads Present Temperature register, returns integer value, actual temperature value equal (1:1). Warning: For MX servo, temperature sensor on PCB, not motor. Actual motor temperature may be higher than reported.
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PRESENT_TEMPERATURE;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_PRESENT_TEMPERATURE;
    printf("Reading Present Temperature\n");
    uint8_t *presTemp = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, address, presTemp, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    printf("Present Temperature read\n");
    int valTemp = 0;
    valTemp = (valTemp << 8) + presTemp[0];

    if (valTemp < 0 || valTemp > 100) {							// Present Temperature valid range: 0 - 100, outside range -> Error
        printf("Error! Value is outside valid range! Unexpected error!\n");
        return -1;
    }
    return valTemp;			// Temperature value ~= Actual Temperature, 1:1 relation
}

int DXLServo::checkPresentCurrent() {
    int address;
    if (servoType == DXL_MX_64)  address = ADDR_MX_PRESENT_CURRENT;
    else if (servoType == DXL_PRO_M42)   address = ADDR_PRO_PRESENT_CURRENT;

    if (limitCurrent <= 0) {													// Limit should not be 0 or negative
        printf("Error! Limit has not been set or unexpected error has occurred!\n");
        return -1;
    }
    else {
        printf("Reading Present Current\n");
        uint16_t *presCur = new uint16_t;
        dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, address, presCur, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }

        printf("Present Current read\n");
        int current = 0;
        current = ((current << 16) + presCur[0]) | ((current << 8) + presCur[1]);

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
    bool success = true;
    int address, limit;
    //double mult;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_PRESENT_CURRENT;
        limit = 1941;
        //mult = 0.00336;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_PRESENT_CURRENT;
        limit = 32767;
        //mult = 0.004028;
    }

    printf("Reading Present Current\n");
    uint16_t *presCur = new uint16_t;
    dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, address, presCur, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Present Current read\n");
        int current = 0;
        current = ((current << 16) + presCur[0]) | ((current << 8) + presCur[1]);

        if (current < 0 || current > limit) {
            printf("Error! Value is outside valid range! Unexpected Error!\n");
            return -1;
        }

        double actCurr = static_cast<double>(current);
        actCurr = convertValtoCurr(current);
        //actCurr = actCurr * mult;   // Actual Current in amps
        return actCurr;
    }
    return 0.0; //if problem
}

int DXLServo::getHomingOffset() {			// Read value of Homing Offset set in servo, returns direct integer value.
    bool success = true;
    int address;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_HOMING_OFFSET;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_HOMING_OFFSET;
    }

    printf("Reading Homing Offset\n");
    int homingOffset = 0;
    dxl_comm_result = this->pktHandler->read4ByteTxRx(this->prtHandler, identity, address, (uint32_t*)&homingOffset, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Homing Offset read\n");
        homeOffset = homingOffset;
    }
    return homingOffset;
}

bool DXLServo::isMoving() {				// Moving check for servo, for use with write position functions to ensure subsequent write does not occur while servo still in motion.
    int address;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_MOVING;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_MOVING;
    }

    uint8_t *move = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, address, move, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int valMove = 0;
    valMove = (valMove << 8) + move[0];

    if (valMove > 0)    return true;
    else    return false;
}

int DXLServo::convertPostoVal(double angle) {			// Convert position in degrees to integer value for transmission
    //int factor,
    int valuePos;
    if (servoType == DXL_MX_64) {
        //factor = 4096;
        valuePos = int( (angle * (4095.0/360.0)) + 0.5);   // 0 - 360 degrees
    }
    else if (servoType == DXL_PRO_M42) {
        //factor = 131593;
        valuePos = int( (angle * (131593.0/180.0)) +0.5); // -180 - +180 degrees
    }

    return valuePos;
}

double DXLServo::convertValtoPos(int position) {		// Convert integer value from transmission to position in degrees
    double angle;
    if (servoType == DXL_MX_64) {
        angle = double(position) * (360.0/4095.0);   // 0 - 4095
    }
    else if (servoType == DXL_PRO_M42) {
        angle = double(position) * (180.0/131593.0); // -131,593 - +131,593
    }

    return angle;
}

int DXLServo::convertCurrtoVal(double current) {			// Convert current in amps to integer value for transmission
    int valueCurr;
    if (servoType == DXL_MX_64) {
        valueCurr = int( (current/0.00336) + 0.5);   // 3.36mA units, 0 - 1941
    }
    else if (servoType == DXL_PRO_M42) {
        //valueCurr = int( (current / 0.004028) + 0.5);	 // 4.028mA units, 0 - 32767, should be limited by Torque Limit
        valueCurr = int( (current / (8250.0/2048.0)) + 0.5);	 // 4.028mA units, 0 - 32767, should be limited by Torque Limit
    }

    if (valueCurr > limitCurrent) {
        printf("ERROR! Current limit exceeded!\n");
        return limitCurrent;
    }
    return valueCurr;
}

double DXLServo::convertValtoCurr(int value) {		// Convert integer value from transmission to current in amps
    double current;
    if (servoType == DXL_MX_64) {
        current = double(value) * 0.00336;   // 3.36mA units, 0 - ~ 4 amps
    }
    else if (servoType == DXL_PRO_M42) {
        //current = double(value) * 0.004028;	 // 4.028mA units, 0 - ~ 2.1 amps
        current = double(value) * (8250.0/2048.0);	 // 4.028mA units, 0 - ~ 2.1 amps
    }

    return current;
}

int DXLServo::convertVeltoVal(double velocity) {			// Convert velocity in rpm to integer value for transmission
    int valueVel;
    if (servoType == DXL_MX_64) {
        valueVel = int((velocity / 0.229) + 0.5);   // 0.229rpm units, 0 - 1023
    }
    else if (servoType == DXL_PRO_M42) {
        valueVel = int((velocity / 0.00389076) + 0.5);	 // 0.00389076rpm units, 0 - 2^32, should be limited by Velocity Limit
    }

    if (valueVel > limitVel) {
        printf("ERROR! Velocity limit exceeded!\n");
        return limitVel;
    }
    return valueVel;
}

double DXLServo::convertValtoVel(int value) {				// Convert integer value from transmission to velocity in rpm
    double velocity;
    if (servoType == DXL_MX_64) {
        velocity = double(value) * 0.229;   // 0.229rpm units
    }
    else if (servoType == DXL_PRO_M42) {
        velocity = double(value) * 0.00389076;	 // 0.00389076rpm units
    }

    return velocity;
}

int DXLServo::convertAcctoVal(double accel) {				// Convert acceleration in rpm2 to integer value for transmission
    int valueAcc;
    if (servoType == DXL_MX_64) {
        valueAcc = int((accel / 214.577) + 0.5);   // 214.577rpm2 units, 0 - 32767
    }
    else if (servoType == DXL_PRO_M42) {
        //valueAcc = int((accel / 201.039) + 0.5);	 // 201.039rpm2 units, 0 - 2^32, should be limited by Acceleration Limit
        valueAcc = int((accel / (58000.0/288.5)) + 0.5);	 // 201.039rpm2 units, 0 - 2^32, should be limited by Acceleration Limit
    }

    if (valueAcc > limitAccel) {
        printf("ERROR! Velocity limit exceeded!\n");
        return limitAccel;
    }
    return valueAcc;
}

double DXLServo::convertValtoAcc(int value) {			// Convert integer value from transmission to acceleration in rpm2
    double accel;
    if (servoType == DXL_MX_64) {
        accel = double(value) * 214.577;   // 214.577rpm2 units
    }
    else if (servoType == DXL_PRO_M42) {
        //accel = double(value) * 201.039;	 // 201.039rpm2 units
        accel = double(value) * (58000.0/288.5);	 // 201.039rpm2 units
    }

    return accel;
}

void DXLServo::servoReboot() {
    // Try reboot. Only for servos on Protocol 2.0.
    // Dynamixel LED will flicker while it reboots
    if (protocolVersion != 2.0) {
        printf("Reboot only available for Protocol 2.0!\nPower down servo to reboot in Protocol 1.0!\n");
        return;
    }

    dxl_comm_result = this->pktHandler->reboot(this->prtHandler, identity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        this->pktHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
        this->pktHandler->printRxPacketError(dxl_error);
    }
}

void DXLServo::selectExtPortMode(int port, int mode) {			// Inputs: port: 1 - 4, select port number; mode: 0 - 3, select port function: 0 - Analog Input mode, 1 - Output mode, 2 - Pull-up Input mode, 3 - Pull-up Output mode
    if (servoType == DXL_MX_64) {
        printf("Error! External Port function not available on MX servos!\n");
        return;
    }

    int address;
    switch (port) {
    case 1: {
        address = ADDR_PRO_EXT_PORT_MODE_1;
        break;
    }
    case 2: {
        address = ADDR_PRO_EXT_PORT_MODE_2;
        break;
    }
    case 3: {
        address = ADDR_PRO_EXT_PORT_MODE_3;
        break;
    }
    case 4: {
        address = ADDR_PRO_EXT_PORT_MODE_4;
        break;
    }
    default:
        printf("Error! Invalid Port number selected! Choose between 1 - 4!\n");
        return;
    }

    uint8_t extmode = uint8_t(mode);
    string function;
    switch (mode) {
    case 0: {
        function = "Analog Input";
        break;
    }
    case 1: {
        function = "Output";
        break;
    }
    case 2: {
        function = "Pull-up Input";
        break;
    }
    case 3: {
        function = "Pull-up Output";
        break;
    }
    default:
        printf("Error! Invalid Mode number selected! Choose between 0 - 3!\n");
        return;
    }

    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, address, extmode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    cout << "External Port " << port << " set to " << function << " mode." << endl;
    externalPort[port - 1] = mode;
}

void DXLServo::setExtPortData(int port, int data) {				// Inputs: port: 1 - 4, select port number; data: 0 or 1, for Output and Pull-up Output Modes only, 0 for 0V, 1 for 3.3V
    if (servoType == DXL_MX_64) {
        printf("Error! External Port function not available on MX servos!\n");
        return;
    }

    if (externalPort[port] == 0 || externalPort[port] == 3) {
        printf("Error! External Port is in Input Mode! Read only!\n");
        return;
    }

    bool success = true;
    int address;
    switch (port) {
    case 1: {
        address = ADDR_PRO_EXT_PORT_DATA_1;
        break;
    }
    case 2: {
        address = ADDR_PRO_EXT_PORT_DATA_2;
        break;
    }
    case 3: {
        address = ADDR_PRO_EXT_PORT_DATA_3;
        break;
    }
    case 4: {
        address = ADDR_PRO_EXT_PORT_DATA_4;
        break;
    }
    default:
        printf("Error! Invalid Port number selected! Choose between 1 - 4!\n");
        return;
    }

    dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if (success) {
        printf("Port %i Data set to %i.\n", port, data);
    }
}

int DXLServo::readExtPortData(int port) {				// Input: port: External Port select number, 1 - 4
    if (servoType == DXL_MX_64) {
        printf("Error! External Port function not available on MX servos!\n");
        return -1;
    }

    bool success = true;
    int address;
    switch (port) {
    case 1: {
        address = ADDR_PRO_EXT_PORT_DATA_1;
        break;
    }
    case 2: {
        address = ADDR_PRO_EXT_PORT_DATA_2;
        break;
    }
    case 3: {
        address = ADDR_PRO_EXT_PORT_DATA_3;
        break;
    }
    case 4: {
        address = ADDR_PRO_EXT_PORT_DATA_4;
        break;
    }
    default:
        printf("Error! Invalid Port number selected! Choose between 1 - 4!\n");
        return -1;
    }

    uint16_t *pass = new uint16_t;
    dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, address, pass, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        success = false;
    }

    if(success){
        printf("External Data %i read\n", port);
        int extData = 0;
        extData = ((extData << 16) + pass[0]) | ((extData << 8) + pass[1]);
        return extData;
    }

    return 0;
}

void  DXLServo::setLED(int color, int light) {
    int address, limit;
    string select;
    switch (color) {
    case 0: {
        address = ADDR_MX_LED;
        limit = 1;
        select = "Single";
        break;
    }
    case 1: {
        address = ADDR_PRO_LED_RED;
        limit = 255;
        select = "Red";
        break;
    }
    case 2: {
        address = ADDR_PRO_LED_GREEN;
        limit = 255;
        select = "Green";
        break;
    }
    case 3: {
        address = ADDR_PRO_LED_BLUE;
        limit = 255;
        select = "Blue";
        break;
    }
    default:
        printf("Error! Invalid Port number selected! Choose between 1 - 4!\n");
        return;
    }

    if (light > limit)		light = limit;
    if (light < 0) {
        printf("Error! Negative numbers not allowed!\n");
        return;
    }

    uint8_t pass = uint8_t(light);
    dxl_comm_result = this->pktHandler->write1ByteTxRx(this->prtHandler, identity, address, pass, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    cout << select << " LED set to " << light << endl;
}

void DXLServo::setPositionGain(int gainP, int gainI, int gainD, int gainF1, int gainF2) {			// gainI, gainD, gainF1, gainF2 default to 0, gainP required in code.
    bool success = true;
    if (servoType == DXL_PRO_M42) {
        printf("Setting Position P Gain\n");
        dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, ADDR_PRO_POSITION_P_GAIN, gainP, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
            success = false;
        }

        if (success) {
            printf("Position P Gain set to %i\n", gainP);
        }
    }
    else if (servoType == DXL_MX_64) {
        int address[5] = { ADDR_MX_POSITION_P_GAIN, ADDR_MX_POSITION_I_GAIN, ADDR_MX_POSITION_D_GAIN, ADDR_MX_POSITION_FF1_GAIN, ADDR_MX_POSITION_FF2_GAIN };
        int gain[5] = { gainP, gainI, gainD, gainF1, gainF2 };
        //char out[5] = { 'P', 'I', 'D', 'FF1', 'FF2' };
        char out[5] = { 'P', 'I', 'D', '1', '2' };

        printf("Setting Position Gains for P, I, D, FF1st, FF2nd\n");
        for (int i = 0; i < 5; i++) {
            dxl_comm_result = this->pktHandler->write2ByteTxRx(this->prtHandler, identity, address[i], gain[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
                success = false;
            }

            if (success) {
                printf("Position %c Gain set to %i\n", out[i], gain[i]);
            }
        }
    }
    else {
        printf("Error! Servo type undefined!\n");
    }
}

void DXLServo::readPositionGain(vector<int> &setGains) {			// Input: setGains: vector of ints for the gain values read. MX will return vector of 5 ints, Pro will return vector of single int. Change to overload, if int return int, if vector return vector?
    bool success = true;
    //uint16_t *presGain = new uint16_t;
    if (servoType == DXL_PRO_M42) {			// Only 1 gain value, read and store into vector once
        printf("Setting Position P Gain\n");
        uint16_t *presGain = new uint16_t;
        dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, ADDR_PRO_POSITION_P_GAIN, presGain, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
        }

        printf("Present Current read\n");
        int gainP = 0;
        gainP = ((gainP << 16) + presGain[0]) | ((gainP << 8) + presGain[1]);

        if (success) {
            printf("Position P Gain set to %i\n", gainP);
            setGains.push_back(gainP);
        }
    }
    else if (servoType == DXL_MX_64) {			// 5 gain values to read and store into vector
        int address[5] = { ADDR_MX_POSITION_P_GAIN, ADDR_MX_POSITION_I_GAIN, ADDR_MX_POSITION_D_GAIN, ADDR_MX_POSITION_FF1_GAIN, ADDR_MX_POSITION_FF2_GAIN };
        //int gain[5] = { 0, 0, 0, 0, 0 };
        //char out[5] = { 'P', 'I', 'D', 'FF1', 'FF2' };
        char out[5] = { 'P', 'I', 'D', '1', '2' };

        printf("Setting Position Gains for P, I, D, FF1st, FF2nd\n");
        for (int i = 0; i < 5; i++) {
            uint16_t *presGain = new uint16_t;
            dxl_comm_result = this->pktHandler->read2ByteTxRx(this->prtHandler, identity, address[i], presGain, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
            }

            //printf("Present Current read\n");
            int gain = 0;
            gain = ((gain << 16) + presGain[0]) | ((gain << 8) + presGain[1]);

            if (success) {
                printf("Position %c Gain set to %i\n", out[i], gain);
                setGains.push_back(gain);
            }
            //presGain = 0;		// reset uint for next loop
        }
    }
    else {
        printf("Error! Servo type undefined!\n");
    }
}

int DXLServo::checkShutdown(int statusBits) {
    int address;
    bool hardErr = false;
    if (servoType == DXL_MX_64) {
        address = ADDR_MX_HARDWARE_ERROR_STATUS;
    }
    else if (servoType == DXL_PRO_M42) {
        address = ADDR_PRO_HARDWARE_ERROR_STATUS;
    }

    uint8_t *status = new uint8_t;
    dxl_comm_result = this->pktHandler->read1ByteTxRx(this->prtHandler, identity, address, status, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->pktHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->pktHandler->getRxPacketError(dxl_error));
    }

    int valStatus = 0;
    valStatus = (valStatus << 8) + status[0];
    statusBits = valStatus;

    if (valStatus > 0) {
        printf("Hardware Error detected! Servo in Shutdown! Checking error...\n");
        hardErr = true;
    }
    else {
        printf("No error detetcted, continue.\n");
        return 0;
    }

    //unsigned int statusBits = (unsigned int)status;
    if (hardErr) {
        if ( ( valStatus & 0x04) == 1 ) {		// bit 2 set means Overheating Error. Typical procedure is to power off and let cool. However, forums describe MX servos temperature sensor to be on pcb instead of motor so by the time the error is detected, motor is likely to have burnt out.
            printf("Overheating Error Detected! Do NOT reboot servo!\nDisconnect power from servo and leave for at least half hour!\n");
            if(servoType == DXL_MX_64)	printf("MX Overheat Error may mean that motor has burnt out. Replacement may be necessary.\n");
            // TOADD: disable servoReboot() command for half hour. Just put thread to sleep for 30 mins?
            return -1;
        }
        else {		// If not Overheat Error, can try reboot. TOADD: Identify individual errors and suggest remedies.
            servoReboot();
            return 1;
        }
    }
    return 0;
}

////////////////////////////////////////////////////   End of DXLServo class   /////////////////////////////////////////////////////////////////////////////////////////////

/*
////////////////////
//		29 Mar 2018 changes not yet implemented in example.
//      21 May: Linux test
//      7 June: MX and Pro functionality. Pro functions untested (awaiting Pro servo).
////////////////////

////////////////////////////////////////////////////    Example Code Using DXLServo functions    ///////////////////////////////////////////////////////////////////////////
int main() {
    // Initialize Servo objects and parameters

    DXLServo panServo;								// DXLServo object 1
    if (panServo.getDXLID() != 1) {				// Ensure Pan set to ID 1
        panServo.setDXLID(1);
    }
    panServo.setDeviceName(0);						// USB port ttyUSB0
    panServo.setDXLServo(0);                        // MX-64 servo in use

    DXLServo tiltServo;								// DXLServo object 2
    if (tiltServo.getDXLID() != 2) {				// Ensure Tilt set to ID 2
        tiltServo.setDXLID(2);
    }
    tiltServo.setDeviceName(0);						// USB port ttyUSB0
    tiltServo.setDXLServo(0);                        // MX-64 servo in use

    // Open USB Port in Linux. ADD: Windows version if necessary
    int fd = 0;
    fd = open(panServo.deviceName.c_str(), O_CREAT | O_RDWR | O_NOCTTY | O_NDELAY);
    //fd = open(panServo.deviceName.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1)    cout << "unable to open USB" << endl;
    else    cout << "USB Port Opened" << endl;

    //initialize Port and Packet handlers, open port, set baudrate
    panServo.initPortHandler();
    panServo.initPacketHandler();
    panServo.openPort();
    panServo.setPortBaudRate();

    // TOCHANGE: Merge all port and packet handler calls into one class as only one instance needed per U2D2, not per servo.
    tiltServo.initPortHandler();
    tiltServo.initPacketHandler();
    tiltServo.openPort();
    tiltServo.setPortBaudRate();

    cout << "Ports Opened" << endl;


    //EEPROM Write, do before enabling Torque
    tiltServo.setOperatingMode(2);  //Set operating mode to Position Mode, internal = 3
    tiltServo.setMaxCurrentLimit();  //Set Current Limit to internally defined max limit for each servo type; 4A for MX, 2.1A for Pro M42
    tiltServo.set80rpmVelLimit();   // Set velocity limit to 80 rpm via internal defined value
    int velLimit = tiltServo.getVelocityLimit();
    cout << "Velocity Limit: " << velLimit << " rpm." << endl;
    tiltServo.setVelocityLimit(80.0);   // Set velocity limit to 80 rpm using double conversion function
    velLimit = tiltServo.getVelocityLimit();
    cout << "Velocity Limit: " << velLimit << " rpm." << endl;
    tiltServo.setlowAccelLimit();   // Set accel limit to internally defined value
    int accLimit = tiltServo.getAccelLimit();
    cout << "Accel Limit: " << accLimit << " rev/min2." << endl;
    tiltServo.setAccelLimit(6400.0);    // Set accel limit to 6400 rpm2 via double conversion
    accLimit = tiltServo.getAccelLimit();
    cout << "Accel Limit: " << accLimit << " rev/min2." << endl;
    //tiltServo.setCurrentLimit(3.5);    //Set current limit to 3.5A, val = 1042
    //tiltServo.setCurrentLimit(1200);    //Set current Limit to val = 1200, 4.032A


    // Enable Torque outputs
    panServo.enableTorque();
    tiltServo.enableTorque();
    cout << "Torque Enabled" << endl;

    //int pan = 0, tilt = 0;
    int currentPanPos = 0, currentTiltPos = 0, currentPanTemp = 0, currentTiltTemp = 0, homePanOffset = 0, homeTiltOffset = 0;
    double currentPanI = 0, currentTiltI = 0;
    int cycle = 3;

    while (cycle > 0) {
        //printf("Press any key to continue! (or press ESC to quit!)\n");
        //if (getch() == ESC_ASCII_VALUE) break;

        // std::cout << "Hello.\nPress enter to continue." << std::endl;
        //std::cin.get();
        //sleep(2);
        crossSleep(1);		// Linux, sleep 1s

        // Read current Pan position and offset
        currentPanPos = panServo.readCurrentPosition();
        homePanOffset = panServo.getHomingOffset();
        cout << "Pan Offset: " << homePanOffset << "." << endl;
        double anglePan = double(currentPanPos + homePanOffset) * 0.088;
        //double anglePan = ( (double)currentPanPos * 0.088 ) - 180 - (homePanOffset * 0.088);
        cout << "Pan Position: " << anglePan << " degrees." << endl;
        double angleP = panServo.readCurrentAngle();
        cout << "Pan Angle = " << angleP << " degrees" << endl;

        // Read current Tilt position and offset
        currentTiltPos = tiltServo.readCurrentPosition();
        homeTiltOffset = tiltServo.getHomingOffset();
        cout << "Tilt Offset: " << homeTiltOffset << "." << endl;
        cout << "Tilt Pos Val: " << currentTiltPos << "." << endl;
        double angleTilt = double(currentTiltPos + homeTiltOffset) * 0.088;
        //double angleTilt = ( (double)currentTiltPos * 0.088 ) - 180 - (homeTiltOffset * 0.088);
        cout << "Tilt Position: " << angleTilt << " degrees." << endl;
        double angleT = tiltServo.readCurrentAngle();
        cout << "Tilt Angle = " << angleT << " degrees" << endl;

        // Read present temperature of servo
        currentPanTemp = panServo.getPresentTemperature();
        cout << "Pan Temp: " << currentPanTemp << "C." << endl;
        currentTiltTemp = tiltServo.getPresentTemperature();
        cout << "Tilt Temp: " << currentTiltTemp << "C." << endl;

        // Read present current through servo. Undefined before 1st move
        currentPanI = panServo.getPresentCurrent();
        cout << "Pan Current: " << std::fixed << std::setprecision(2) << currentPanI << "A." << endl;
        currentTiltI = tiltServo.getPresentCurrent();
        cout << "Tilt Current: " << std::fixed << std::setprecision(2) << currentTiltI << "A." << endl;

        // Set Profile Acceleration and Velocity then read their respective registers
        tiltServo.setProfileAcceleration(5);
        int profileAccel = tiltServo.checkProfileAcceleration();
        cout << "Profile Acc Val: " << profileAccel << " ." << endl;
        tiltServo.setProfileVelocity(10);
        int profileVel = tiltServo.checkProfileVelocity();
        cout << "Profile Vel Val: " << profileVel << " ." << endl;

        // Read the internal gains of the servo
        vector<int> gains;
        gains.clear();
        tiltServo.readPositionGain(gains);
        cout << "Position Gains are: P = " << gains[0] << ", I = " << gains[1] << ", D = " << gains[2] << ", FF1 = " << gains[3] << ", FF2 = " << gains[4] << "." << endl;;
        gains.clear();

        // GoalPosition function test
        tiltServo.writeGoalPosition(1);
        std::vector<int> tiltGoals;
        tiltGoals.clear();
//        tiltGoals.push_back(1830);
//        tiltGoals.push_back(1900);
//        tiltGoals.push_back(1700);
        int start = 1603;
        for(int i = 0; i < 10; i++){
            tiltGoals.push_back(start);
            start += 45;
        }
        tiltServo.addToGoalPosVector(tiltGoals);
        int checkGPV = tiltServo.checkGoalPosVector();
//        tiltServo.writeGoalPosition(1);
//        //sleep(2);
//        crossSleep(1);		// Linux, sleep 1s
//        angleT = tiltServo.readCurrentAngle();
//        cout << "Tilt Angle = " << angleT << " degrees" << endl;
//        tiltServo.writeGoalPosition(1, 2000);
//        //sleep(2);
//        crossSleep(1);		// Linux, sleep 1s
//        angleT = tiltServo.readCurrentAngle();
//        cout << "Tilt Angle = " << angleT << " degrees" << endl;
//        tiltServo.writeGoalPosition(0, 2);
//        //sleep(2);
//        crossSleep(1);		// Linux, sleep 1s
//        angleT = tiltServo.readCurrentAngle();
//        cout << "Tilt Angle = " << angleT << " degrees" << endl;
        for(int x = 0; x < 10; x++){
            tiltServo.writeGoalPosition(x);
            crossSleep(1);		// Linux, sleep 1s
            angleT = tiltServo.readCurrentAngle();
            cout << "Tilt Angle = " << angleT << " degrees" << endl;
        }

        tiltGoals.clear();
        tiltServo.resetGoalPosVector();

        //GoalAngle function test
        vector<double> goalAngles;
        goalAngles.clear();
//        goalAngles.push_back(0.0);
//        goalAngles.push_back(6.15);
//        goalAngles.push_back(-11.43);
        double start1 = -20.0;
        for(int i = 0; i < 10; i++){
            goalAngles.push_back(start1);
            start1 += 4.0;
        }
        tiltServo.addToGoalAngleVect(goalAngles);
        int checkGAV = tiltServo.checkGoalAngleVect();
//        tiltServo.writeGoalAngle(1);
//        crossSleep(1);
//        angleT = tiltServo.readCurrentAngle();
//        cout << "Tilt Angle = " << angleT << " degrees" << endl;
//        tiltServo.writeGoalAngle(1, 14.95);
//        crossSleep(1);
//        angleT = tiltServo.readCurrentAngle();
//        cout << "Tilt Angle = " << angleT << " degrees" << endl;
//        tiltServo.writeGoalAngle(0, 2);
//        crossSleep(1);
//        angleT = tiltServo.readCurrentAngle();
//        cout << "Tilt Angle = " << angleT << " degrees" << endl;
        for(int x = 0; x < 10; x++){
            tiltServo.writeGoalAngle(x);
            crossSleep(1);		// Linux, sleep 1s
            angleT = tiltServo.readCurrentAngle();
            cout << "Tilt Angle = " << angleT << " degrees" << endl;
        }

        goalAngles.clear();
        tiltServo.resetGoalAngleVect();

        cycle -= 1;
        cout << "Next Cycle" << endl << endl;
    }

    // Close DXLServo

    // Clear internal position vectors
    panServo.resetGoalPosVector();
    tiltServo.resetGoalPosVector();

    // Disable DXLs Torque outputs
    panServo.disableTorque();
    tiltServo.disableTorque();
    cout << "Torque Disabled" << endl;

    // Close Comms port
    panServo.closePort();
    tiltServo.closePort();
    close(fd);
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

} while ((abs(dxl1_goal_position[index1] - dxl1_present_position) > DXL_MX_MOVING_STATUS_THRESHOLD) || (abs(dxl2_goal_position[index2] - dxl2_present_position) > DXL_MX_MOVING_STATUS_THRESHOLD));

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

