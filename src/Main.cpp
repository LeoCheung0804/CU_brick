#pragma comment(lib, "User32.lib")
#include "../include/Logger.h"
#include "../include/Robot.h"
#include "../include/TrajectoryGenerator.h"
#include <iostream>
#include <string>
#include <fstream>
#include "../tools/json.hpp"
#include "../include/utils.h"

#pragma comment(lib, "ws2_32.lib")

using json = nlohmann::json;
using namespace std;

string userInput;
Robot robot;
Logger logger;

void ShutdownRobotUntilSafe(const string& reason){
    logger.LogError(reason);
    cerr << reason << endl;
    while(!robot.Disconnect()){
        logger.LogError(
            "Robot shutdown incomplete; controller connection remains active."
        );
        cerr << "CRITICAL: Secure the hardware, then press a key to retry "
             << "shutdown." << endl;
        system("pause");
    }
}

bool RequireAutomatedGripperCommand(
    bool commandSucceeded,
    const string& description
){
    if(commandSucceeded) return true;
    ShutdownRobotUntilSafe(
        "Automated gripper command failed (" + description
        + "); aborting controls."
    );
    return false;
}

void DrawBricksASCII(int brickType){
    if(brickType  == 1){
        cout << "  ########################  " << endl;
        cout << "  #     ---      ---     #" << endl;
        cout << "  #    /   \\    /   \\    #" << endl;
        cout << "  #    \\   /    \\   /    #" << endl;
        cout << "  #     ---      ---     #" << endl;
        cout << "  ########################  " << endl;
    }else if(brickType == 2){   
        cout << "  ########################  " << endl;
        cout << "  #                      #" << endl;
        cout << "  #                      #" << endl;
        cout << "  #                      #" << endl;
        cout << "  #                      #" << endl;
        cout << "  ########################  " << endl;
    }else if(brickType == 3){
        cout << "  #########  " << endl;
        cout << "  #       #" << endl;
        cout << "  #       #" << endl;
        cout << "  #       #" << endl;
        cout << "  #########  " << endl;
    }
}

void ClearConsoleInputBuffer()
{
    // If you happen to have any trouble clearing already cleared buffer, uncomment the section below.
    /* keybd_event('S', 0, 0, 0);
    keybd_event('S', 0,KEYEVENTF_KEYUP, 0);
    keybd_event(VK_BACK, 0, 0, 0);
    keybd_event(VK_BACK, 0,KEYEVENTF_KEYUP, 0); */
    PINPUT_RECORD ClearingVar1 = new INPUT_RECORD[256];
    DWORD ClearingVar2;
    ReadConsoleInput(GetStdHandle(STD_INPUT_HANDLE),ClearingVar1,256,&ClearingVar2);
    delete[] ClearingVar1;
}

void PrintGripperControlMenu(){
    system("CLS");
    cout << "====================== Gripper Control Menu ======================" << endl;
    cout << "\t1 - Close Gripper" << endl;
    cout << "\t2 - Open Gripper" << endl;
    cout << "\t3 - Rotate Gripper" << endl;
    cout << "\t4 - Release Gripper" << endl;
    cout << "\t5 - Write Calibration Data to Gripper" << endl;
    cout << "\t6 - Reset" << endl;
    cout << "\t7 - Reconnect" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void GripperControlMode(){    
    while(true){
        if(!robot.IsConnected()) return;
        PrintGripperControlMenu();
        cin >> userInput;
        if(userInput == "q"){
            cout << "Bye" << endl;
            break;
        }else if(userInput == "1"){
            if(!robot.gripper.Close())
                cerr << "Gripper close command failed." << endl;
            system("pause");
        }else if(userInput == "2"){
            if(!robot.gripper.Open())
                cerr << "Gripper open command failed." << endl;
            system("pause");
        }else if(userInput == "3"){
            while(true){
                cout << "Please Enter Target Angle (q to exit, x/y to set start/end angle for calibration): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else if(userInput == "x"){
                    cout << "Please Enter Start Angle for Calibration (Enter None Integer to exit): ";
                    cin >> userInput;
                    char* p;
                    int angle = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        robot.gripper.SetCalibrateStartAngle(angle);
                    }else{
                        cout << "Please enter intager!!!";
                    }
                }else if(userInput == "y"){
                    cout << "Please Enter Start Angle for Calibration (Enter None Integer to exit): ";
                    cin >> userInput;
                    char* p;
                    int angle = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        robot.gripper.SetCalibrateEndAngle(angle);
                    }else{
                        cout << "Please enter intager!!!";
                    }
                }else{
                    char* p;
                    int angle = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        if(!robot.gripper.Rotate(angle))
                            cerr << "Gripper rotation command failed." << endl;
                    }else{
                        cout << "Please enter intager!!!";
                    }
                }
            }
        }else if(userInput == "4"){
            if(!robot.gripper.Release())
                cerr << "Gripper release command failed." << endl;
            system("pause");
        }else if(userInput == "5"){
            robot.gripper.Calibrate();
            system("pause");
        }else if(userInput == "6"){
            robot.gripper.Reset();
            system("pause");
        }else if(userInput == "7"){
            if(!robot.gripper.Reconnect()){
                cerr << "Failed to reconnect gripper." << endl;
                ShutdownRobotUntilSafe(
                    "Gripper reconnect failed; forcing fail-safe robot shutdown."
                );
                return;
            }
            system("pause");
        }
    }
}

int selectedCable = 0;
void PrintCableMotorControlMenu(){
    system("CLS");
    cout << "====================== Cable Motor Control Mode  ======================" << endl;
    cout << "Beware that there is no soft emergency stop in this mode. " << endl;
    cout << "Current Selected Cable: " << selectedCable << endl;
    cout << "\t1 - Select Cable" << endl;
    cout << "\t2 - Set Cable Trq" << endl;
    cout << "\t3 - Move Selected Cable Relative (Motor Step)" << endl;
    cout << "\t4 - Move Selected Cable Relative (Cable Length)" << endl;
    cout << "\t5 - Move ALL Cable Relative (Motor Step)" << endl;
    cout << "\t6 - Move ALL Cable Relative (Cable Length)" << endl;
    // cout << "\t7 - Move Selected Cable Absolute (Motor Step)" << endl;
    // cout << "\t8 - Move Selected Cable Absolute (Cable Length)" << endl;
    // cout << "\t9 - Move ALL Cable Absolute (Motor Step)" << endl;
    // cout << "\t10 - Move ALL Cable Absolute (Cable Length)" << endl;
    // cout << "\t11 - Open Selected Cable Brake" << endl;
    // cout << "\t12 - Close Selected Cable Brake" << endl;
    // cout << "\t13 - Open ALL Cable Brake" << endl;
    // cout << "\t14 - Close ALL Cable Brake" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void CableMotorControlMode(){
    while(true){
        if(!robot.IsConnected()) return;
        PrintCableMotorControlMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Select Cable
            while(true){
                cout << "Please Enter Cable Index (0-"
                     << robot.GetCableMotorNum() - 1 << "): ";
                cin >> userInput;
                char* p;
                int index = strtol(userInput.c_str(), &p, 10);
                if(!*p && index >= 0 && index < robot.GetCableMotorNum()) {
                    selectedCable = index;
                    cout << "Cable " << index << " Selected" << endl;
                    break;
                }else{
                    cout << "Please enter a configured cable index." << endl;
                }
            }
            system("pause");
        }else if(userInput == "2"){ // Set Cable Trq
            cout << "Please Enter Target Torque (Default " << robot.GetWorkingTrq() << "): ";
            getchar();
            getline(cin, userInput);
            char* p;
            double targetTrq = strtod(userInput.c_str(), &p);
            if(!*p) {
                if(!userInput.length())
                    targetTrq = robot.GetWorkingTrq();
                if(targetTrq < -10 || targetTrq > 2){
                    cout << "Target Torque Invalid." << endl;
                    system("pause");
                    continue;
                }
                cout << "Setting to Target Torque: " << targetTrq << endl;
                if(!robot.cable.SetCableTrqByIndex(
                    selectedCable,
                    static_cast<float>(targetTrq),
                    4
                )){
                    ShutdownRobotUntilSafe(
                        "Cable torque adjustment could not verify a safe stop."
                    );
                    return;
                }
            }else{
                cout << "Please enter a number !!!" << endl;
            }
            system("pause");
        }else if(userInput == "3"){ // Move Selected Cable Relative By Motor Step
            while(true){
                cout << "Move Selected Cable Relative By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving Cable " << selectedCable << " By Step: " << step << endl;
                        if(step / robot.GetCableMotorScale(selectedCable) > 0.5 || step / robot.GetCableMotorScale(selectedCable) < -0.5){
                            cout << "Step too large! range should be in -0.5m(" << -0.5 * robot.GetCableMotorScale(selectedCable) << ") ~ 0.5m(" << 0.5 * robot.GetCableMotorScale(selectedCable) << ")." << endl;
                            continue;
                        }
                        if(!robot.cable.MoveSingleMotorCmd(selectedCable, step, false)){
                            if(!robot.cable.StopAllMotor()){
                                ShutdownRobotUntilSafe(
                                    "Cable move failed and motors could not be stopped."
                                );
                                return;
                            }
                            cerr << "Cable move failed; remaining commands aborted."
                                 << endl;
                            break;
                        }
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "4"){ // Move Selected Cable Relative By Cable Length
            while(true){
                cout << "Move Selected Cable Relative By Cable Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Cable " << selectedCable << " By Length: " << length << endl;
                        if(length > 0.5 || length < -0.5){
                            cout << "Step too large! range should be in -0.5 ~ 0.5." << endl;
                            continue;
                        }
                        if(!robot.cable.MoveSingleMotorCmd(
                            selectedCable,
                            robot.CableMotorLengthToCmd(selectedCable, length),
                            false
                        )){
                            if(!robot.cable.StopAllMotor()){
                                ShutdownRobotUntilSafe(
                                    "Cable move failed and motors could not be stopped."
                                );
                                return;
                            }
                            cerr << "Cable move failed; remaining commands aborted."
                                 << endl;
                            break;
                        }
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "5"){ // Move ALL Cable Relative By Motor Step
            while(true){
                cout << "Move ALL Cable Relative By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving ALL Cable By Step: " << step << endl;
                        bool stepIsValid = true;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            if(step / robot.GetCableMotorScale(i) > 0.5 || step / robot.GetCableMotorScale(i) < -0.5){
                                cout << "Step too large! range should be in -0.5m(" << -0.5 * robot.GetCableMotorScale(i) << ") ~ 0.5m(" << 0.5 * robot.GetCableMotorScale(i) << ")." << endl;
                                stepIsValid = false;
                                break;
                            }
                        }
                        if(!stepIsValid)
                            continue;
                        if(!robot.cable.MoveAllMotorCmd(step, false)){
                            if(!robot.cable.StopAllMotor()){
                                ShutdownRobotUntilSafe(
                                    "Cable group move failed and motors could not be stopped."
                                );
                                return;
                            }
                            cerr << "Cable group move failed and was stopped." << endl;
                            break;
                        }
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "6"){ // Move ALL Cable Relative By Cable Length
            while(true){
                cout << "Move ALL Cable Relative By Cable Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Cable By Length: " << length << endl;
                        if(length > 0.5 || length < -0.5){
                            cout << "Step too large! range should be in -0.5m(" << -0.5 * robot.GetCableMotorScale(selectedCable) << ") ~ 0.5m(" << 0.5 * robot.GetCableMotorScale(selectedCable) << ")." << endl;
                            continue;
                        }
                        vector<int32_t> commands;
                        commands.reserve(
                            static_cast<size_t>(robot.GetCableMotorNum())
                        );
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            commands.push_back(
                                robot.CableMotorLengthToCmd(i, length)
                            );
                        }
                        if(!robot.cable.MoveAllMotorCmd(commands, false)){
                            if(!robot.cable.StopAllMotor()){
                                ShutdownRobotUntilSafe(
                                    "Cable group move failed and motors could not be stopped."
                                );
                                return;
                            }
                            cerr << "Cable group move failed and was stopped." << endl;
                            break;
                        }
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        // }else if(userInput == "7"){ // Move Selected Cable Absolute By Motor Step
        //     while(true){
        //         cout << "Move Selected Cable Absolute By Motor Step" << endl;
        //         cout << "Please Enter Step (enter q to exit): ";
        //         cin >> userInput;
        //         if(userInput == "q"){
        //             break;
        //         }else{
        //             char* p;
        //             int step = strtol(userInput.c_str(), &p, 10);
        //             if(!*p) {
        //                 cout << "Moving Cable " << selectedCable << " To Absolute Step: " << step << endl;
        //                 robot.cable.MoveSingleMotorCmd(selectedCable, step, true);
        //             }else{
        //                 cout << "Please enter an intager number!!!" << endl;
        //             }
        //         }
        //     }
        // }else if(userInput == "8"){ // Move Selected Cable Absolute By Cable Length
        //     while(true){
        //         cout << "Move Selected Cable Absolute By Cable Length" << endl;
        //         cout << "Please Enter Length in Meters (enter q to exit): ";
        //         cin >> userInput;
        //         if(userInput == "q"){
        //             break;
        //         }else{
        //             char* p;
        //             double length = strtod(userInput.c_str(), &p);
        //             if(!*p) {
        //                 cout << "Moving Cable " << selectedCable << " To Absolute Length: " << length << endl;
        //                 robot.cable.MoveSingleMotorCmd(selectedCable, robot.CableMotorLengthToCmd(length), true);
        //             }else{
        //                 cout << "Please enter a number!!!" << endl;
        //             }
        //         }
        //     }
        // }else if(userInput == "9"){ // Move ALL Cable Absolute By Motor Step
        //     while(true){
        //         cout << "Move ALL Cable Absolute By Motor Step" << endl;
        //         cout << "Please Enter Step (enter q to exit): ";
        //         cin >> userInput;
        //         if(userInput == "q"){
        //             break;
        //         }else{
        //             char* p;
        //             int step = strtol(userInput.c_str(), &p, 10);
        //             if(!*p) {
        //                 cout << "Moving ALL Cable To Absolute Step: " << step << endl;
        //                 robot.cable.MoveAllMotorCmd(step, true);
        //             }else{
        //                 cout << "Please enter an intager number!!!" << endl;
        //             }
        //         }
        //     }
        // }else if(userInput == "10"){ // Move ALL Cable Absolute By Cable Length
        //     while(true){
        //         cout << "Move ALL Cable Absolute By Cable Length" << endl;
        //         cout << "Please Enter Length in Meters (enter q to exit): ";
        //         cin >> userInput;
        //         if(userInput == "q"){
        //             break;
        //         }else{
        //             char* p;
        //             double length = strtod(userInput.c_str(), &p);
        //             if(!*p) {
        //                 cout << "Moving ALL Cable To Absolute Length: " << length << endl;
        //                 robot.cable.MoveAllMotorCmd(robot.CableMotorLengthToCmd(length), true);
        //             }else{
        //                 cout << "Please enter a number!!!" << endl;
        //             }
        //         }
        //     }
        }
    }

}

void PrintRailMotorControlMenu(int selectedRailMotor){
    system("CLS");
    cout << "====================== Rail Motor Control Mode  ======================" << endl;
    cout << "Beware that there is no soft emergency stop in this mode. " << endl;
    cout << "Current Selected Rail: " << selectedRailMotor << endl;
    cout << "\t1 - Select Rail" << endl;
    cout << "\t2 - Move Selected Rail Absolute (Motor Step)" << endl;
    cout << "\t3 - Move Selected Rail Absolute (Rail Length)" << endl;
    cout << "\t4 - Move ALL Rail Absolute (Motor Step)" << endl;
    cout << "\t5 - Move ALL Rail Absolute (Rail Length)" << endl;
    cout << "\t6 - Open Selected Rail Brake" << endl;
    cout << "\t7 - Close Selected Rail Brake" << endl;
    cout << "\t8 - Open ALL Rail Brake" << endl;
    cout << "\t9 - Close ALL Rail Brake" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void RailMotorControlMode(){
    int selectedRailMotor = 0;
    while(true){
        if(!robot.IsConnected()) return;
        PrintRailMotorControlMenu(selectedRailMotor);
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Select Rail
            while(true){
                cout << "Please Enter Rail Index (0-"
                     << robot.GetRailMotorNum() - 1 << "): ";
                cin >> userInput;
                char* p;
                int index = strtol(userInput.c_str(), &p, 10);
                if(!*p
                    && index >= 0
                    && index < robot.GetRailMotorNum()
                    && robot.rail.SelectWorkingMotor(index)) {
                    selectedRailMotor = index;
                    cout << "Rail " << index << " Selected" << endl;
                    break;
                }else{
                    cout << "Please enter a valid rail index." << endl;
                }
            }
            system("pause");
        }else if(userInput == "2"){ // Move Selected Rail Absolute By Motor Step
            while(true){
                cout << "Move Selected Rail Absolute By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRailMotor << " To Absolute Step: " << step << endl;
                        if(robot.rail.MoveSelectedMotorCmdAbsulote(step)){
                            robot.railOffset[selectedRailMotor] = step;
                        }else{
                            ShutdownRobotUntilSafe(
                                "Rail move failed; aborting controls."
                            );
                            return;
                        }
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "3"){ // Move Selected Rail Absolute By Rail Length
            while(true){
                cout << "Move Selected Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p && std::isfinite(length)) {
                        cout << "Moving Rail " << selectedRailMotor << " To Absolute Length: " << length << endl;
                        if(robot.rail.MoveSelectedMotorCmdAbsulote(
                            robot.RailMotorLengthToCmd(selectedRailMotor, length)
                        )){
                            robot.railOffset[selectedRailMotor] = length;
                        }else{
                            ShutdownRobotUntilSafe(
                                "Rail move failed; aborting controls."
                            );
                            return;
                        }
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "4"){ // Move ALL Rail Absolute By Motor Step
            while(true){
                cout << "Move ALL Rail Absolute By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving ALL Rail To Absolute Step: " << step << endl;
                        for(int i = 0; i < robot.GetRailMotorNum(); i++){
                            if(!robot.rail.MoveMotorCmdAbsuloteById(i, step)){
                                ShutdownRobotUntilSafe(
                                    "Rail group move failed; aborting controls."
                                );
                                return;
                            }
                        }
                        if(!robot.rail.SelectWorkingMotor(selectedRailMotor)){
                            ShutdownRobotUntilSafe(
                                "Failed to restore selected rail motor."
                            );
                            return;
                        }
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "5"){ // Move ALL Rail Absolute By Rail Length
            while(true){
                cout << "Move ALL Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p && std::isfinite(length)) {
                        cout << "Moving ALL Rail To Absolute Length: " << length << endl;
                        for(int i = 0; i < robot.GetRailMotorNum(); i++){
                            if(!robot.rail.MoveMotorCmdAbsuloteById(
                                i,
                                robot.RailMotorLengthToCmd(i, length)
                            )){
                                ShutdownRobotUntilSafe(
                                    "Rail group move failed; aborting controls."
                                );
                                return;
                            }
                            robot.railOffset[i] = length;
                        }
                        if(!robot.rail.SelectWorkingMotor(selectedRailMotor)){
                            ShutdownRobotUntilSafe(
                                "Failed to restore selected rail motor."
                            );
                            return;
                        }
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "6"){ // Open Selected Rail Brake
            cout << "Opening Rail " << selectedRailMotor << " Brake" << endl;
            if(!robot.brake.OpenRailBrakeByIndex(selectedRailMotor)){
                ShutdownRobotUntilSafe(
                    "Failed to open selected rail brake; aborting controls."
                );
                return;
            }
            system("pause");
        }else if(userInput == "7"){ // Close Selected Rail Brake
            cout << "Closing Rail " << selectedRailMotor << " Brake" << endl;
            if(!robot.brake.CloseRailBrakeByIndex(selectedRailMotor)){
                ShutdownRobotUntilSafe(
                    "Failed to close selected rail brake; aborting controls."
                );
                return;
            }
            system("pause");
        }else if(userInput == "8"){ // Open ALL Rail Brake
            cout << "Opening ALL Rail Brake" << endl;
            if(!robot.brake.OpenAllRailBrake()){
                ShutdownRobotUntilSafe(
                    "Failed to open all rail brakes; aborting controls."
                );
                return;
            }
            system("pause");
        }else if(userInput == "9"){ // Close ALL Rail Brake
            cout << "Closing ALL Rail Brake" << endl;
            if(!robot.brake.CloseAllRailBrake()){
                ShutdownRobotUntilSafe(
                    "Failed to close all rail brakes; aborting controls."
                );
                return;
            }
            system("pause");
        }
    }

}

void PrintRobotPosControlMenu(){
    system("CLS");
    cout << "====================== Robot Pos Control Mode  ======================" << endl;
    cout << "Beware that there is no soft emergency stop in this mode. " << endl;
    cout << "\tw - Move the robot on +y direction." << endl;
    cout << "\ta - Move the robot on +x direction." << endl;
    cout << "\ts - Move the robot on -y direction." << endl;
    cout << "\td - Move the robot on -x direction." << endl;
    cout << "\tr - Move the robot on +z direction." << endl;
    cout << "\tf - Move the robot on -z direction." << endl;
    cout << "\tq - Exit Robot Pos Control Mode." << endl;
    cout << "=====================================================================" << endl;
}

void RobotPosControlMode(){
    PrintRobotPosControlMenu();
    robot.PrintEEPos();
    double goalPos[6] = {0};
    int sleepTime = 120;
    int moveTime = 100;
    while(true){
        if(!robot.IsConnected()) return;
        if(GetKeyState('Q') & 0x8000)
        {
            Sleep(100);
            ClearConsoleInputBuffer();
            break;
        }else if(GetKeyState('A') & 0x8000) // +x
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[0] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('D') & 0x8000) // -x
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[0] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('W') & 0x8000) // +y
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[1] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('S') & 0x8000) // -y
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[1] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('R') & 0x8000) // +z
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[2] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('F') & 0x8000) // -z
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[2] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else{
            Sleep(sleepTime);
        }
    }
}

void PrintRailControlMenu(int selectedRail){
    system("CLS");
    cout << "====================== Rail Control Mode  ======================" << endl;
    cout << "Current Selected Rail: " << selectedRail << endl;
    cout << "\t1 - Select Rail" << endl;
    cout << "\t2 - Move Selected Rail Absolute (Rail Length)" << endl;
    cout << "\t3 - Move ALL Rail Absolute (Rail Length)" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void RailControlMode(){
    
    int selectedRail = 0;
    while(true){
        if(!robot.IsConnected()) return;
        PrintRailControlMenu(selectedRail);
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Select Rail
            while(true){
                cout << "Please Enter Rail Index (0-"
                     << robot.GetRailMotorNum() - 1 << "): ";
                cin >> userInput;
                char* p;
                int index = strtol(userInput.c_str(), &p, 10);
                if(!*p
                    && index >= 0
                    && index < robot.GetRailMotorNum()
                    && robot.rail.SelectWorkingMotor(index)) {
                    selectedRail = index;
                    cout << "Rail " << index << " Selected" << endl;
                    break;
                }else{
                    cout << "Please enter a valid rail index." << endl;
                }
            }
            system("pause");
        }else if(userInput == "2"){ // Move Selected Rail Absolute By Rail Length
            while(true){
                cout << "Move Selected Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p && std::isfinite(length)) {
                        cout << "Moving Rail " << selectedRail << " To Absolute Length: " << length << " Count: " << robot.RailMotorLengthToCmd(selectedRail, length) << endl;
                        // int cableIndex = (selectedRail + robot.railMotorCableMotorOffset) % 4;
                        if(robot.RaiseRailWithCableByLengthAbsulote(
                            selectedRail,
                            selectedRail,
                            static_cast<float>(length)
                        )){
                            logger.LogInfo("Raise rail " + to_string(selectedRail) + " to " + to_string(length));
                        }else{
                            logger.LogError("Failed to raise rail " + to_string(selectedRail));
                            if(!robot.IsConnected()){
                                ShutdownRobotUntilSafe(
                                    "Coordinated rail move left shutdown state unverified."
                                );
                                return;
                            }
                        }
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "3"){ // Move ALL Rail Absolute By Rail Length
            while(true){
                cout << "Move ALL Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p && std::isfinite(length)) {
                        cout << "Moving ALL Rail To Absolute Length: " << length << " Count: " << robot.RailMotorLengthToCmd(selectedRail, length) << endl;
                        for(int i = 0; i < robot.GetRailMotorNum(); i++){
                            if(!robot.rail.SelectWorkingMotor(i)){
                                logger.LogError("Failed to select rail " + to_string(i));
                                break;
                            }
                            // int cableIndex = (i + robot.railMotorCableMotorOffset) % 4;
                            if(!robot.RaiseRailWithCableByLengthAbsulote(
                                i,
                                i,
                                static_cast<float>(length)
                            )){
                                logger.LogError("Failed to raise rail " + to_string(i));
                                if(!robot.IsConnected()){
                                    ShutdownRobotUntilSafe(
                                        "Coordinated rail move left shutdown state unverified."
                                    );
                                    return;
                                }
                                break;
                            }
                            logger.LogInfo("Raise rail " + to_string(i) + " to " + to_string(length));
                        }
                        if(!robot.rail.SelectWorkingMotor(selectedRail))
                            logger.LogError("Failed to restore selected rail motor.");
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }
    }
}

void PrintRobotControlMenu(){
    system("CLS");
    cout << "====================== Robot Control Mode  ======================" << endl;
    cout << "\t1 - Robot Position Control" << endl;
    cout << "\t2 - Move Robot to Home Position" << endl;
    cout << "\t3 - Move Robot to Pre-Pickup Position" << endl;
    cout << "\t4 - Gripper Control" << endl;
    cout << "\t5 - Rail Control " << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void RobotControlMode(){
    while(true){
        if(!robot.IsConnected()) return;
        PrintRobotControlMenu();
        cin >> userInput;

        if(userInput == "q"){ // back to last page
            break;
        }else if(userInput == "1"){ // robot position control mode
            // robot.brake.OpenAllCableBrake();
            RobotPosControlMode();
            // robot.brake.CloseAllCableBrake();
        }else if(userInput == "2"){ // move robot to home position
            robot.PrintHomePos();
            // robot.brake.OpenAllCableBrake();
            robot.MoveToParaBlend(robot.homePos, true);
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "3"){ // move robot to pre-pickup position
            // robot.PrintPrePickupPos();
            // robot.brake.OpenAllCableBrake();
            double goalPos[6] = {0};
            copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
            goalPos[5] = 0.0141; // calculated yaw for +0.21 height
            goalPos[2] += 0.21; // 0.21 safe height from ABB
            robot.MoveToParaBlend(goalPos, true);
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "4"){ // gripper control mode
            GripperControlMode();
            if(!robot.IsConnected()) return;
        }else if(userInput == "5"){ // Rail Control
            // robot.brake.OpenAllCableBrake();
            if(!robot.brake.OpenAllRailBrake()){
                ShutdownRobotUntilSafe(
                    "Rail control aborted: rail brakes could not be opened."
                );
                return;
            }
            RailControlMode();
            if(!robot.IsConnected()) return;
            // robot.brake.CloseAllCableBrake();
            if(!robot.brake.CloseAllRailBrake()){
                ShutdownRobotUntilSafe(
                    "Failed to close all rail brakes after rail control."
                );
                return;
            }
        }
    }
}

void PrintCalibrationMenu(){
    system("CLS");
    cout << "====================== Calibration Mode  ======================" << endl;
    cout << "\t1 - Set Cables Torque" << endl;
    cout << "\t2 - Request current cable motor torque readings" << endl;
    cout << "\t3 - Control Cable Motor " << endl;
    cout << "\t4 - Update Robot Pos" << endl;
    cout << "\t5 - Reset EE Rotation to Zero" << endl;
    cout << "\t6 - Control Linear Rail Motor" << endl;
    cout << "\t7 - Control Gripper " << endl;
    cout << "\t8 - Update Robot Config" << endl;
    cout << "\t9 - Print Robot Status " << endl;
    cout << "\t10 - Clear Exception " << endl;
    cout << "\t11 - Robot Control Mode" << endl;
    cout << "\t12 - Request Current Position from RPi" << endl;
    cout << "\tq - Finish Calibration" << endl;
    cout << "Please Select Operation: " << endl;
}

void CalibrationMode(){ 
    while(true){
        if(!robot.IsConnected()) return;
        PrintCalibrationMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Set Cable Trq
            cout << "Please Enter Target Torque (Default " << robot.GetWorkingTrq() << "): ";
            getchar();
            getline(cin, userInput);
            char* p;
            double targetTrq = strtod(userInput.c_str(), &p);
            if(!*p) {
                if(!userInput.length())
                    targetTrq = robot.GetWorkingTrq();
                if(targetTrq < -10 || targetTrq > 2){
                    cout << "Target Torque Invalid." << endl;
                    system("pause");
                    continue;
                }
                cout << "Setting to Target Torque: " << targetTrq << endl;
                // robot.brake.OpenAllCableBrake();
                if(!robot.cable.SetCableTrq(
                    static_cast<float>(targetTrq),
                    4
                )){
                    ShutdownRobotUntilSafe(
                        "Cable torque adjustment could not verify a safe stop."
                    );
                    return;
                }
                // robot.brake.CloseAllCableBrake();
            }else{
                cout << "Please enter a number !!!" << endl;
            }
            system("pause");
        }else if(userInput == "2"){ // requst current torque readings 
            cout << "Current Measured Cable Motor Trq: " << endl;
            // robot.brake.OpenAllCableBrake();
            for(int i = 0; i < robot.GetCableMotorNum(); i++){
                cout << "\tCable " << i << ": " << robot.cable.GetMotorTorqueMeasured(i) << endl;
            }
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "3"){ // Control Cable Individual
            // robot.brake.OpenAllCableBrake();
            CableMotorControlMode();
            // robot.brake.CloseAllCableBrake();
        }else if(userInput == "4"){ // Update Robot Pos By File
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            if(!robot.UpdatePosFromFile(
                robotPosPath != "" ? robotPosPath : "lastPos.txt"
            )){
                ShutdownRobotUntilSafe(
                    "Position-file calibration failed; aborting controls."
                );
                return;
            }
            logger.LogPos(robot.endEffectorPos, robot.railOffset);
            if(!robot.CheckLimits()){
                logger.LogWarning("New Position is beyond robot limit.");
                cout << "Warning: New Position is beyond robot limit. " << endl;
            }
            system("pause");
        }else if(userInput == "5"){ // Reset End Effector Rotation to 0,0,0
            double targetPos[6] = { robot.endEffectorPos[0], robot.endEffectorPos[1], robot.endEffectorPos[2], 0, 0, 0 };
            // robot.brake.OpenAllCableBrake();
            robot.MoveToParaBlend(targetPos, 3500, true);
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "6"){ // Control Linear Rail Motor
            if(!robot.brake.OpenAllRailBrake()){
                ShutdownRobotUntilSafe(
                    "Rail control aborted: rail brakes could not be opened."
                );
                return;
            }
            RailMotorControlMode();
            if(!robot.IsConnected()) return;
            if(!robot.brake.CloseAllRailBrake()){
                ShutdownRobotUntilSafe(
                    "Failed to close all rail brakes after rail control."
                );
                return;
            }
        }else if(userInput == "7"){ // Control Gripper
            GripperControlMode();
            if(!robot.IsConnected()) return;
        }else if(userInput == "8"){ // Update Robot Config
            string robotConfigPath;
            cout << "Please Enter Robot Config File Path (Default RobotConfig.json): ";
            getchar();
            getline(cin, robotConfigPath);
            bool configUpdated = robot.UpdateModelFromFile(
                robotConfigPath != "" ? robotConfigPath : "RobotConfig.json",
                true
            );
            while(!configUpdated && !robot.IsValid()){
                cout << "Config file invalid, please use correct config file." << endl;
                string robotConfigFile;
                cout << "Please Enter Robot Configuration File Path (Default RobotConfig.json): ";
                getline(cin, robotConfigFile);
                configUpdated = robot.UpdateModelFromFile(
                    robotConfigFile != "" ? robotConfigFile : "RobotConfig.json",
                    true
                );
            }
            if(!configUpdated || !robot.IsConnected()){
                ShutdownRobotUntilSafe(
                    "Robot reconnect after configuration update failed."
                );
                return;
            }
            system("pause");
        }else if(userInput == "9"){ // print current status
            robot.PrintEEPos();
            robot.PrintRailOffset();
            robot.PrintCableLength();
            system("pause");
        }else if(userInput == "10"){ // clear motor alert
            robot.cable.ClearAlert();
            system("pause");
        }else if(userInput == "11"){ // Robot Control Mode
            RobotControlMode();
        }else if(userInput == "12"){ // Request Current Position from RPi
            WSADATA wsaData;
            if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
                cout << "Failed to initialize Winsock" << endl;
                return ;
            }

            // Create UDP socket
            bool rpiWinsockInitialized = true;
            SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            const auto cleanupRPiSocket = [&udpSocket, &rpiWinsockInitialized](){
                if(udpSocket != INVALID_SOCKET){
                    if(closesocket(udpSocket) == SOCKET_ERROR){
                        cout << "Failed to close RPi socket" << endl;
                        return false;
                    }
                    udpSocket = INVALID_SOCKET;
                }
                if(rpiWinsockInitialized && WSACleanup() == SOCKET_ERROR){
                    cout << "Failed to release Winsock" << endl;
                    return false;
                }
                rpiWinsockInitialized = false;
                return true;
            };
            const auto cleanupRPiSocketUntilReleased = [&cleanupRPiSocket](){
                while(!cleanupRPiSocket()){
                    cout << "RPi network cleanup is incomplete; retrying." << endl;
                    system("pause");
                }
            };
            if (udpSocket == INVALID_SOCKET) {
                cout << "Failed to create socket" << endl;
                cleanupRPiSocketUntilReleased();
                system("pause");
                continue;
            }

            // Set up server address
            sockaddr_in serverAddr;
            serverAddr.sin_family = AF_INET;
            serverAddr.sin_port = htons(robot.RPiPort); // Use appropriate port number
            serverAddr.sin_addr.s_addr = inet_addr(robot.RPiIP.c_str()); // Use RPi's IP address

            // Send request
            const char* request = "get_position";
            if (sendto(udpSocket, request, strlen(request), 0, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
                cout << "Failed to send request" << endl;
                cleanupRPiSocketUntilReleased();
                system("pause");
                continue;
            }

            // Receive response
            char buffer[1024];
            int serverAddrLen = sizeof(serverAddr);
            int bytesReceived = recvfrom(
                udpSocket,
                buffer,
                sizeof(buffer) - 1,
                0,
                (sockaddr*)&serverAddr,
                &serverAddrLen
            );
            
            if (bytesReceived == SOCKET_ERROR) {
                cout << "Failed to receive response" << endl;
                cleanupRPiSocketUntilReleased();
                system("pause");
                continue;
            }

            // Null terminate the received data
            buffer[bytesReceived] = '\0';
            cout << "Received position data: " << buffer << endl;

            // Close socket
            cleanupRPiSocketUntilReleased();
            // parse the buffer to json
            json jsonData;
            try{
                jsonData = json::parse(buffer);
            }catch(const json::parse_error& e){
                cout << "Error parsing JSON: " << e.what() << endl;
                system("pause");
                return;
            }
            cout << "==========================================================" << endl;
            cout << "Data from RPi | Current Position: " << endl;
            cout << "x: " << jsonData["x"] << " | " << robot.endEffectorPos[0] << endl;
            cout << "y: " << jsonData["y"] << " | " << robot.endEffectorPos[1] << endl;
            cout << "z: " << jsonData["z"] << " | " << robot.endEffectorPos[2] << endl;
            cout << "roll: " << jsonData["roll"] << " | " << robot.endEffectorPos[3] << endl;
            cout << "pitch: " << jsonData["pitch"] << " | " << robot.endEffectorPos[4] << endl;
            cout << "yaw: " << jsonData["yaw"] << " | " << robot.endEffectorPos[5] << endl;
            cout << "Update robot position? (y/n): ";
            cin >> userInput;
            if(userInput == "y"){
                // update robot position
                double measuredPos[6] = {jsonData["x"], jsonData["y"], jsonData["z"], jsonData["roll"], jsonData["pitch"], jsonData["yaw"]};
                if(!robot.UpdatePos(measuredPos)){
                    ShutdownRobotUntilSafe(
                        "Position calibration failed; aborting controls."
                    );
                    return;
                }
            }
        }
    }
}

void PrintOperationMenu(){
    system("CLS");
    cout << "====================== Operation Mode  ======================" << endl;
    cout << "\t1 - Robot Control" << endl;
    cout << "\t2 - Read Brick Positions File" << endl;
    cout << "\t3 - Read Point-to-Point Path File" << endl;
    cout << "\t4 - Read Trajectory File" << endl;
    cout << "\t5 - Request current cable motor torque readings" << endl;
    cout << "\t6 - Save Current EE Pos to File" << endl;
    cout << "\t7 - Print Robot Status " << endl; 
    cout << "\t8 - Demo Mode" << endl;
    cout << "\t9 - Auto Aiming Mode" << endl;
    cout << "\t10 - Drone Mode" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

//bool CheckContinue(){
//    std::string userInput;
//    while(true){
//        std::cout << "Continue? (Y/n): ";
//        std::getline(std::cin, userInput);
//        if(userInput == "" || userInput == "y" || userInput == "Y"){
//            return true;
//        }
//        if(userInput == "n" || userInput == "N") {
//            return false;
//        }
//    }
//}

void OperationMode(){
    while(true){
        if(!robot.IsConnected()) return;
        PrintOperationMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Robot Pos Control
            RobotControlMode();
        }else if(userInput == "2"){ // Read brick position file
            string brickPosFileName;
            cout << "Please Enter Brick Pos File Path (Default bricks.csv): ";
            getchar();
            getline(cin, brickPosFileName);
            vector<vector<double>> brickPosList = ReadBrickPosFile(brickPosFileName != "" ? brickPosFileName : "bricks.csv", robot.rotationalAngleOffset, robot.rotationalDistanceOffset); // Read "bricks.csv"
            if(brickPosList.size() == 0){
                system("pause");
                continue;  
            } 
            cout << "The file contains " << brickPosList.size() << " bricks." << endl;
            cout << "Please Enter Next Brick Index (Default 0): ";
            getline(cin, userInput);
            char* p;
            int brickIndex = strtol((userInput != "" ? userInput : "0").c_str(), &p, 10);
            if(*p){
                cout << "Please enter intager!!!";
                continue;
            }
            robot.PrintEEPos();
            robot.PrintBrickPickUpPos();
            cout << "Speed Limit: " << robot.GetEffVelLmt() << "m/s" << endl;
            double goalPos[6] = {0};
            if(!robot.brake.OpenAllCableBrake()){
                ShutdownRobotUntilSafe(
                    "Brick operation aborted: cable brakes could not be opened."
                );
                return;
            }
            double safePt[6] = {8.24, 6.51, -0.6, 0, 0, -0.0237}; // a safe area near to the arm // 0.21 safe height from ABB
            bool stop = false;
            string brickTypes[4] = {"??Should not show this, please check", "holes", "normal", "half"};
            int brickType = 0;
            for(int i = brickIndex; i < brickPosList.size(); i++){
            // for(vector<double> brickPos : brickPosList){
                vector<double> brickPos = brickPosList[i];
                brickType = int(brickPos[5]);
                // Move to pre pick up position
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Rotate(90),
                    "pre-pickup rotation"
                )) return;
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Open(),
                    "pre-pickup open"
                )) return;

                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                goalPos[5] = 0.0141; // calculated yaw for +0.21 height
                goalPos[2] += 0.21; // 0.21 safe height from ABB

                cout << "Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                DrawBricksASCII(brickType);
                cout << "====== Moving to pre pick up position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;

                Sleep(800); // wait for ee to start moving
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                
                Sleep(100); // wait for brick

                // lower the ee to pickup brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                cout << "Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Picking up brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT * 1.2, true)) break;

                // Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Close(),
                    "brick pickup close"
                )) return;
                Sleep(800); // wait for grippper to close

                // raise brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos+6, begin(goalPos));
                goalPos[2] += 0.14;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Raise up brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(800); // wait for grippper to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                // move to safe point                
                copy(safePt, safePt+6, begin(goalPos)); // safe point
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to safe position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, true)) break;

                // rotate gripper to target rotation
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Rotate(
                        static_cast<int>(
                            brickPos[4] + 92.2
                            - brickPos[3] / 3.1415965 * 180
                        )
                    ),
                    "brick placement rotation"
                )) return;

                double safeH = 0.3; // meter, safety height from building brick level
                // avoid the 5th pole
                /*
                double lowPole = -2.9; // lowest 5th pole z-value
                double safeZoneH = 0.4; // height of safe zone from bottom cable to top cable near EE
                if(brickPos[1]+2.149848*brickPos[0]>25.250178){ // check if target brick pos falls into the region where cables may hit 5th pole
                    cout << "Attention: Brick is in mountain region. Caution: may hit 5th pole\n"; 
                    // near road, check if 5th pole posistion need moving
                    if((lowPole-brickPos[2])*(lowPole-brickPos[2]+safeZoneH)<=0){
                        // Move 5th pole to just above the safe zone
                        // MoveBrkRail(brickPos[2]+safeZoneH);
                        goalPos[2] = lowPole; // safe level as 5th pole
                        cout << "Lowering to pole position??\n";
                        if(!robot.MoveToParaBlend(goalPos, false)) break;
                    }
                }
                else if(brickPos[1]-2.271415*brickPos[0]<-13.260362){ // check if target brick pos falls into the region where cables may hit 5th pole
                    cout << "Attention: Brick is in brick loading region. Caution: may hit 5th pole\n";
                    if((lowPole-brickPos[2])*(lowPole-brickPos[2]+safeZoneH)<=0){
                        // if lowPole < brickPos[2]+EeH { move ee to 5th pole height || move 5th above EE safeZone}
                        // else {move 5th pole to ee height}
                        // move to 5th pole height
                        goalPos[2] = lowPole; // safe level as 5th pole
                        cout << "Lowering to pole position??\n";
                        if(!robot.MoveToParaBlend(goalPos, false)) break;
                    }
                }
                // double safeH = 0.12; // meter, safety height from building brick level
                // goalPos[2] = brickPos[2] + robot.GetEEToGroundOffset() + safeH.; // brick level with safe height
                // if(robot.endEffectorPos[2] < brickPos[2] + robot.GetEEToGroundOffset() + safeH){ // if current position is below target brick height, then raise brick first // need this?
                //     if(!robot.MoveToParaBlend(goalPos, false)) break;
                // }
                */
                
                if(robot.endEffectorPos[2] < brickPos[2] + robot.GetEEToGroundOffset() + safeH){ // if current position is below target brick height, then raise brick first // need this?
                    goalPos[0] = robot.endEffectorPos[0];
                    goalPos[1] = robot.endEffectorPos[1];
                    goalPos[2] = brickPos[2] + safeH + robot.GetEEToGroundOffset();
                    cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                    cout << "====== Rise Brick before Moving to brick position." << endl;
                    cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                    cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                    if(!CheckContinue()) break;
                    if(!robot.MoveToParaBlend(goalPos, true)) break;
                }
                // Go to brick placing position
                goalPos[0] = brickPos[0];
                goalPos[1] = brickPos[1];
                goalPos[2] = brickPos[2] + safeH + robot.GetEEToGroundOffset();
                goalPos[5] = brickPos[3]; // Include yaw angle
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to brick position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(800); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                
                // Place brick
                goalPos[2] -= safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Placing brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for grippper to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Open(),
                    "brick placement open"
                )) return;
                Sleep(200); //Wait a while after placing brick

                // Rise and leave building area
                goalPos[2] += safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to stand by position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                // move to safe point
                copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y,z position
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to safe position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(robot.endEffectorPos[2] > safePt[2]){ // if current position is above safe point, then return to point above safe point then lowering z-height
                    goalPos[2] = robot.endEffectorPos[2];
                    if(!robot.MoveToParaBlend(goalPos, true)) break;
                }                
                goalPos[2] = safePt[2];
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                
                cout << "----------Completed brick #" << i <<"----------" << endl;
                if(!CheckContinue()) break;
            }
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "3"){ // read point to point file, gen and run para blend traj
            string ptnFileName;
            cout << "Please Enter Point-To-Point Path File Path (Default points.csv): ";
            getchar();
            getline(cin, ptnFileName);
            vector<vector<double>> pointList = ReadPointFile(ptnFileName != "" ? ptnFileName : "points.csv"); // Read "bricks.csv"
            cout << "The file contains " << pointList.size() << " points." << endl;
            if(!CheckContinue()) continue;
            // robot.brake.OpenAllCableBrake();
            for(vector<double> point : pointList){
                if(!robot.MoveToParaBlend(&point[0], true)) break;
            }
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "4"){ // read and run traj file
            string trajFileName;
            cout << "Please Enter Trajectory File Path (Default traj.csv): ";
            getchar();
            getline(cin, trajFileName);
            vector<vector<double>> trajList = ReadTrajFile(trajFileName != "" ? trajFileName : "traj.csv"); // Read "bricks.csv"
            cout << "The file contains " << trajList.size() << " points." << endl;
            if(!CheckContinue()) continue;
            // robot.brake.OpenAllCableBrake();
            if(!robot.MoveToParaBlend(&(trajList[0])[0])){
                cout << "Could not move to the trajectory start point; trajectory aborted."
                     << endl;
                if(!robot.IsConnected()){
                    ShutdownRobotUntilSafe(
                        "Trajectory positioning failed and left the robot unavailable."
                    );
                    return;
                }
                system("pause");
                continue;
            }
            if(!robot.RunCableTraj(trajList) && !robot.IsConnected()){
                ShutdownRobotUntilSafe(
                    "Trajectory execution failed and left the robot unavailable."
                );
                return;
            }
            // robot.brake.CloseAllCableBrake();
            system("pause");
        }else if(userInput == "5"){ // requst current torque readings
            cout << "Current Measured Cable Motor Trq: " << endl;
            for(int i = 0; i < robot.GetCableMotorNum(); i++){
                cout << "\tCable " << i << ": " << robot.cable.GetMotorTorqueMeasured(i) << endl;
            }
            system("pause");
        }else if(userInput == "6"){ // save current ee pos to file
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            robot.SavePosToFile(robotPosPath  != "" ? robotPosPath : "lastPos.txt");
            system("pause");
        }else if(userInput == "7"){ // print current status
            robot.PrintEEPos();
            robot.PrintRailOffset();
            robot.PrintCableLength();
            system("pause");
        }else if(userInput == "8"){ // demo mode
            cout << "Demo Mode" << endl;
            string brickPosFileName;
            cout << "Please Enter Brick Pos File Path (Default demoab1.csv): ";
            getchar();
            getline(cin, brickPosFileName);
            vector<vector<double>> brickPosList = ReadBrickPosFile(brickPosFileName != "" ? brickPosFileName : "demoab1.csv", robot.rotationalAngleOffset, robot.rotationalDistanceOffset); // Read "bricks.csv"
            if(brickPosList.size() == 0){
                system("pause");
                continue;  
            } 
            cout << "The file contains " << brickPosList.size() << " bricks." << endl;
            cout << "Please Enter Next Brick Index (Default 0): ";
            getline(cin, userInput);
            char* p;
            int brickIndex = strtol((userInput != "" ? userInput : "0").c_str(), &p, 10);
            if(*p){
                cout << "Please enter intager!!!";
                continue;
            }
            robot.PrintEEPos();
            robot.PrintBrickPickUpPos();
            cout << "Speed Limit: " << robot.GetEffVelLmt() << "m/s" << endl;
            double goalPos[6] = {0};
            if(!robot.brake.OpenAllCableBrake()){
                ShutdownRobotUntilSafe(
                    "Brick operation aborted: cable brakes could not be opened."
                );
                return;
            }
            bool stop = false;
            string brickTypes[4] = {"??Should not show this, please check", "holes", "normal", "half"};
            int brickType = 0;
            for(int i = brickIndex; i < brickPosList.size(); i++){
                vector<double> brickPos = brickPosList[i];
                // Move to pre pick up position
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Rotate(90),
                    "demo pre-pickup rotation"
                )) return;
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Open(),
                    "demo pre-pickup open"
                )) return;

                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                goalPos[2] += 0.2; // 0.2 safe height from ABB
                cout << "Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to pre pick up position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(800); // wait for ee to start moving
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                Sleep(100); // wait for brick

                // lower the ee to pickup brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                cout << "Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Picking up brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT * 1.2, true)) break;

                // Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Close(),
                    "demo brick pickup close"
                )) return;
                Sleep(800); // wait for grippper to close

                // raise brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos+6, begin(goalPos));
                goalPos[2] += 0.14;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Raise up brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(800); // wait for grippper to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;
                // rotate gripper to target rotation
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Rotate(
                        static_cast<int>(
                            brickPos[4] + 92.2
                            - brickPos[3] / 3.1415965 * 180
                        )
                    ),
                    "demo brick placement rotation"
                )) return;
                double safeH = 0.2; // meter, safety height from building brick level
                
                // Go to brick placing position
                goalPos[0] = brickPos[0];
                goalPos[1] = brickPos[1];
                goalPos[2] = brickPos[2] + safeH;
                //goalPos[5] = brickPos[3]; // Include yaw angle
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to brick position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(800); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, true)) break;

                // Place brick
                goalPos[2] -= safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Placing brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for grippper to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Open(),
                    "demo brick placement open"
                )) return;
                Sleep(200); //Wait a while after placing brick

                // Rise and leave building area
                goalPos[2] += safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to stand by position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                // Move down again for pick up
                goalPos[2] -= safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to stand by position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;
                // Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Close(),
                    "demo brick re-grip close"
                )) return;
                Sleep(800); // wait for grippper to close

                // Rise and leave building area
                goalPos[2] += safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to stand by position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                //move to brick pick up position
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                goalPos[2] += 0.2; // 0.2 safe height from ABB
                cout << "Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to pre pick up position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(800); // wait for ee to start moving
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                Sleep(100); // wait for brick

                // Place brick
                goalPos[2] -= safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Placing brick." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for grippper to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                if(!RequireAutomatedGripperCommand(
                    robot.gripper.Open(),
                    "demo return placement open"
                )) return;
                Sleep(200); //Wait a while after placing brick

                // Rise and leave building area
                goalPos[2] += safeH;
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;
                cout << "====== Moving to stand by position." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;

                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                //Move to home position
                copy(robot.homePos, robot.homePos+6, begin(goalPos)); // home x,y,z position
                cout << "====== Brick No. " << i << " (" << brickTypes[brickType] << ")" << endl;   
                cout << "====== Moving to home postition." << endl;
                cout << "Pos: " << "x: " << goalPos[0] << " y: " << goalPos[1] << " z: " << goalPos[2] << endl;
                cout << "Rot: " << "roll: " << goalPos[3] << " pitch: " << goalPos[4] << " yaw: " << goalPos[5] << endl;
                //if(!CheckContinue()) break;
                Sleep(1000); // wait for ee to stop
                if(!robot.MoveToParaBlend(goalPos, robot.safeT * 2, true)) break;
            }
            system("pause");
        }else if(userInput == "9"){ // auto aiming mode
            WSADATA wsaData;
            double* camera_offset = robot.GetCameraOffset(); // camera offset from end effector
            if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
                cout << "Failed to initialize Winsock" << endl;
                return;
            }

            // Create UDP socket
            bool autoAimWinsockInitialized = true;
            SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            const auto cleanupAutoAimSocket =
                [&udpSocket, &autoAimWinsockInitialized](){
                if(udpSocket != INVALID_SOCKET){
                    if(closesocket(udpSocket) == SOCKET_ERROR){
                        cout << "Failed to close auto-aim socket" << endl;
                        return false;
                    }
                    udpSocket = INVALID_SOCKET;
                }
                if(autoAimWinsockInitialized && WSACleanup() == SOCKET_ERROR){
                    cout << "Failed to release Winsock" << endl;
                    return false;
                }
                autoAimWinsockInitialized = false;
                return true;
            };
            const auto cleanupAutoAimSocketUntilReleased =
                [&cleanupAutoAimSocket](){
                while(!cleanupAutoAimSocket()){
                    cout << "Auto-aim network cleanup is incomplete; retrying."
                         << endl;
                    system("pause");
                }
            };
            if (udpSocket == INVALID_SOCKET) {
                cout << "Failed to create socket" << endl;
                cleanupAutoAimSocketUntilReleased();
                system("pause");
                continue;
            }

            // Set up server address
            sockaddr_in serverAddr;
            serverAddr.sin_family = AF_INET;
            // serverAddr.sin_port = htons(robot.RPiPort); // Use appropriate port number
            // serverAddr.sin_addr.s_addr = inet_addr(robot.RPiIP.c_str()); // Use RPi's IP address
            // localhost for testing
            serverAddr.sin_port = htons(5000); // Use appropriate port number
            serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

            cout << "Connecting to RPi at " << robot.RPiIP << ":" << robot.RPiPort << endl;

            double offset[3];
            double angle;
            double goalPos[6];
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            while (true) {
                bool offsetRequestFailed = false;
                for (int i = 0; i < 3; i++) {
                    if (!robot.GetMoveToBrickPos(udpSocket, serverAddr, offset, angle)) {
                        cout << "Failed to get offset from RPi." << endl;
                        offsetRequestFailed = true;
                        break;
                    }
                    cout << "Received offset: x=" << offset[0] << ", y=" << offset[1] << ", z=" << offset[2] << endl;
                    // Move to the new position
                    goalPos[0] += offset[0];
                    goalPos[1] += offset[1];
                    goalPos[2] += offset[2];
                    cout << "Moving to new position: x=" << goalPos[0] << ", y=" << goalPos[1] << ", z=" << goalPos[2] << endl;
                    if(!robot.MoveToParaBlend(goalPos, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim positioning failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    Sleep(500);
                }
                if(offsetRequestFailed) break;

                char userInput;
                cout << "Continue to next offset? (y/n/(g)rip/(p)lace): ";
                cin >> userInput;
                if (userInput == 'n' || userInput == 'N')
                    break;
                else if (userInput == 'g' || userInput == 'G') {
                    cout << "Gripping the brick..." << endl;
                    if(!robot.gripper.Open()){
                        ShutdownRobotUntilSafe(
                            "Auto-aim gripper open failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    goalPos[0] += camera_offset[0];
                    goalPos[1] += camera_offset[1];
                    goalPos[2] += -0.30;
                    if(!robot.MoveToParaBlend(goalPos, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim pickup approach failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    if(!robot.gripper.Rotate(static_cast<int>(-angle))){
                        ShutdownRobotUntilSafe(
                            "Auto-aim gripper rotation failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    Sleep(1000);
                    goalPos[2] += -0.185;
                    if(!robot.MoveToParaBlend(goalPos, 2000, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim pickup descent failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    if(!robot.gripper.Close()){
                        ShutdownRobotUntilSafe(
                            "Auto-aim gripper close failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    Sleep(1000);
                    copy(robot.homePos, robot.homePos+6, begin(goalPos)); // home x,y,z position
                    if(!robot.MoveToParaBlend(goalPos, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim return-home motion failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }

                    break;
                } else if (userInput == 'p' || userInput == 'P') {
                    cout << "Placing the brick..." << endl;
                    if(!robot.gripper.Rotate(static_cast<int>(-angle))){
                        ShutdownRobotUntilSafe(
                            "Auto-aim placement rotation failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    double x = 0.28 * cos((90 - angle) * 3.1415 / 180.0);
                    double y = 0.28 * sin((90 - angle) * 3.1415 / 180.0);
                    cout << "place offset: x=" << x << ", y=" << y << " right side? y/n/(t)op: ";
                    cin >> userInput;
                    if (userInput == 'n' || userInput == 'N') {
                        x = -x; // flip the x offset
                        y = -y; // flip the y offset
                        cout << "place offset: x=" << x << ", y=" << y << endl;
                    } else if (userInput == 't' || userInput == 'T') {
                        // place on top
                        x = 0;
                        y = 0;
                        goalPos[2] += 0.055; // height of a brick
                    }
                    goalPos[0] += x + camera_offset[0];
                    goalPos[1] += y + camera_offset[1];
                    goalPos[2] += -0.30;
                    if(!robot.MoveToParaBlend(goalPos, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim placement approach failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    
                    goalPos[2] += -0.185;
                    if(!robot.MoveToParaBlend(goalPos, 2000, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim placement descent failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    if(!robot.gripper.Open()){
                        ShutdownRobotUntilSafe(
                            "Auto-aim placement open failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    Sleep(1500);
                    goalPos[2] += 0.185;
                    if(!robot.MoveToParaBlend(goalPos, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim placement retreat failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    copy(robot.homePos, robot.homePos+6, begin(goalPos)); // home x,y,z position
                    if(!robot.MoveToParaBlend(goalPos, true)){
                        ShutdownRobotUntilSafe(
                            "Auto-aim return-home motion failed; aborting controls."
                        );
                        cleanupAutoAimSocketUntilReleased();
                        return;
                    }
                    break;
                } else if (userInput != 'y' && userInput != 'Y') {
                    cout << "Invalid input, exiting auto aiming mode." << endl;
                    break;
                }
            }
            // Close socket
            cleanupAutoAimSocketUntilReleased();
        }else if(userInput == "10"){ // Drone Trajectory Mode
            cout << "Drone Trajectory Mode" << endl;
            cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {  // valid number
                        cout << "Moving Bottom 4 Cable By Length: " << length << endl;
                        if(length > 0.5 || length < -0.5){ // limit step size to 0.5m
                            cout << "Step too large! range should be in -0.5m(" << -0.5 * robot.GetCableMotorScale(selectedCable) << ") ~ 0.5m(" << 0.5 * robot.GetCableMotorScale(selectedCable) << ")." << endl;
                            continue;
                        }
                        double goalPos[6] = {0};
                        copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
                        goalPos[2] += length;
                        if(!robot.MoveToParaBlend(goalPos, 3000, true)){
                            if(!robot.IsConnected()){
                                ShutdownRobotUntilSafe(
                                    "Drone motion failed and left the robot unavailable."
                                );
                                return;
                            }
                            cout << "Drone motion was stopped; no further movement will run."
                                 << endl;
                            break;
                        }
                        Sleep(800); // wait for ee to start moving
                        if(!robot.MoveToParaBlend(goalPos, true)){
                            if(!robot.IsConnected()){
                                ShutdownRobotUntilSafe(
                                    "Drone motion failed and left the robot unavailable."
                                );
                                return;
                            }
                            break;
                        }
                        Sleep(100); // waits
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            break;
        }else if(userInput == "q"){ // exit
            break;
            }
    }
}

void PrintMainMenu(){
    system("CLS");
    cout << "====================== Mode Selection  ======================" << endl;
    cout << "\t1 - Calibration Mode" << endl;
    cout << "\t2 - Operation Mode" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void MainMenu(){
    while(true){
        if(!robot.IsConnected()){
            cerr << "Robot connection is no longer ready. Leaving the main menu "
                 << "for fail-safe shutdown." << endl;
            logger.LogError("Robot connection lost; leaving the main menu.");
            break;
        }
        PrintMainMenu();
        cin >> userInput;
        if(userInput == "1"){
            CalibrationMode();
        }else if(userInput == "2"){
            OperationMode();
        }else if(userInput == "q"){
            cout << "Are you sure to exit? (y/n): ";
            cin >> userInput;
            if(userInput == "y" || userInput == "q"){
                break;
            }
        }
    }
}

int main(){
    if(!logger.OpenFile("log\\robot.log")){
        cout << "Cannot open logger file. System will now exit." << endl;
        exit(-1);
    }
    
    logger.LogInfo("Reading robot config.");
    // read robot config file
    if(fstream("RobotConfig.json").good()){
        cout << "RobotConfig.json found. Config robot using RobotConfig.json " << endl;
        robot.UpdateModelFromFile("RobotConfig.json");
    }
    while(!robot.IsValid()){
        cout << "Config file invalid, please use correct config file." << endl;
        string robotConfigFile;
        cout << "Please Enter Robot Configuration File Path (Default RobotConfig.json): ";
        getline(cin, robotConfigFile);
        robot.UpdateModelFromFile(robotConfigFile != "" ? robotConfigFile : "RobotConfig.json");
    }

    
    // connect to robot
    const bool robotConnected = robot.Connect();
    logger.LogInfo("Connecting to robot.");

    if(robotConnected && robot.IsConnected()){
        cout << "All motors, all brakes and gripper connected success." << endl;
        logger.LogInfo("All motors, all brakes and gripper connected success.");

        // Recovery may calibrate live controllers, so it must only run after
        // the complete robot connection has been verified.
        if(fstream("lastPos.txt").good()){
            cout << "lastPos.txt found. Recover last position of robot." << endl;
            if(!robot.UpdatePosFromFile("lastPos.txt", true)){
                ShutdownRobotUntilSafe(
                    "Startup position recovery failed; initialization aborted."
                );
                logger.CloseFile();
                return 1;
            }
            logger.LogInfo("Reading Last Pos from pos log file");
        }
        
        system("pause");
        // start main program
        MainMenu();
    }else{
        cout << "Robot Init Failed. Exiting..." << endl;
        logger.LogError("Robot initialization failed.");
        while(!robot.Disconnect()){
            logger.LogError(
                "Robot initialization rollback is incomplete; "
                "controller connection remains active."
            );
            cerr << "CRITICAL: Secure the hardware, then press a key to retry "
                 << "shutdown." << endl;
            system("pause");
        }
        logger.CloseFile();
        return 1;
    }

    // Only a still-ready operational session may update the recovery file.
    if(robot.IsConnected()){
        robot.PrintEEPos();
        robot.PrintRailOffset();
        string robotPosPath;
        cout << "Please Enter Robot Position File Path (Default lastPos.txt, enter 'no' to escape from saving): ";
        getchar();
        getline(cin, robotPosPath);
        if(robotPosPath != "no"){
            robot.SavePosToFile(robotPosPath  != "" ? robotPosPath : "lastPos.txt");
            system("pause");
        }
    }else{
        logger.LogError(
            "Skipping recovery-file save because the robot is not fully connected."
        );
    }
    
    // disconnect from robot
    while(!robot.Disconnect()){
        logger.LogError("Robot disconnect incomplete; controller connection remains active.");
        cerr << "CRITICAL: Secure the hardware, then press a key to retry "
             << "shutdown." << endl;
        system("pause");
    }
    logger.LogInfo("Robot Disconnected.");
    logger.CloseFile();
    cout << "Bye" << endl;

    return 0;
}
