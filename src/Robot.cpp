#include "../include/Robot.h"
#include "../include/CableKinematics.h"
#include "../tools/json.hpp"
#include "../Dependencies/eigen-3.3.7/Eigen/Dense"
#include "../include/TrajectoryGenerator.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <conio.h>

using json = nlohmann::json;

Robot::Robot() {};

// Constructor, based on the requested model
Robot::Robot(string robotConfigPath)
{
    this->UpdateModelFromFile(robotConfigPath);
}

bool Robot::Connect()
{
    if(this->isConnected
        || this->shutdownIncomplete
        || this->gripper.HasActiveResources()
        || this->brake.HasActiveResources()
        || this->cable.HasActiveResources()
        || this->rail.HasActiveResources()){
        if(!this->Disconnect()){
            cerr << "Error: Existing robot connection could not be safely closed."
                 << endl;
            return false;
        }
    }
    this->shutdownIncomplete = false;

    if (this->isOnline)
    {
        if (!this->posLogger.OpenFile("log\\pos.log"))
        {
            cout << "Cannot open pos logger file. Connection aborted." << endl;
            this->isConnected = false;
            return false;
        }
    }

    this->isConnected = false;
    // Init BLE Nodes
    if (this->useGripper)
    {
        this->gripper = GripperController(this->isOnline, this->useGripper);
        if (!this->gripper.Connect(this->gripperCommPort))
        {
            cout << "Error: Gripper not connected." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Gripper connection failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
    }

    // Init Brake Motor Nodes
    if (this->useRailBraker || this->useCableBraker)
    {
        this->brake = BrakeController(this->isOnline);
        if(this->useCableBraker
            && !this->brake.UseCableBrake(this->cableMotorBrakeNum)){
            cout << "Error: Invalid cable brake configuration." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Brake configuration failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
        if(this->useRailBraker
            && !this->brake.UseRailBrake(this->railMotorNum)){
            cout << "Error: Invalid rail brake configuration." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Brake configuration failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
        if (!this->brake.Connect(this->railBrakeCommPort))
        {
            cout << "Error: Brake not connected." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Brake connection failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
    }

    // Init Rail Motor Nodes
    if (this->useRailMotor)
    {
        this->rail = RailController(this->isOnline, this->useRailMotor);
        if (!this->rail.Connect(851, this->railMotorNum))
        {
            cout << "Error: Rail motors not connected." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Rail connection failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
        vector<double> railMotorCmd;
        if(!this->rail.GetMotorPosMeasured(railMotorCmd)
            || railMotorCmd.size() != static_cast<size_t>(this->railMotorNum)){
            cerr << "Error: Rail position verification failed during initialization."
                 << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Rail verification failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
        for(int i = 0; i < this->railMotorNum; i++){
            double currentPos = railMotorCmd[i]; // in internal counts
            cout << "Rail " << i << " current pos: " << currentPos << endl;
            // this->rail.MoveMotorCmdAbsuloteById(i, currentPos);
            // this->brake.OpenRailBrakeByIndex(i);
            // int targetPos = this->RailMotorLengthToCmd(i, this->railOffset[i]); // length offset to internal counts
            // cout << "Rail " << i << " target pos: " << targetPos << endl;
            // this->rail.MoveMotorCmdAbsuloteById(i, targetPos);
            // this->brake.CloseRailBrakeByIndex(i);
        }

    }

    // Init Cable Motor Nodes
    if (this->useCableMotor)
    {
        this->cable = CableController(this->isOnline, this->useCableMotor);
        if(!this->brake.OpenAllCableBrake()){
            cout << "Error: Cable brakes could not be opened." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Cable-brake preparation failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
        const bool cableConnected = this->cable.Connect(this->cableMotorNum);
        if(!cableConnected
            && this->cable.HasActiveResources()
            && !this->cable.StopAllMotor()){
            cerr << "CRITICAL: Cable initialization failed and retained motor "
                 << "resources could not be stopped. Brakes remain open for "
                 << "a safe retry." << endl;
            this->isConnected = false;
            this->shutdownIncomplete = true;
            return false;
        }
        const bool cableBrakesClosed = this->brake.CloseAllCableBrake();
        if (!cableConnected || !cableBrakesClosed)
        {
            cout << "Error: Cable motor or brake initialization failed." << endl;
            this->isConnected = false;
            if(!this->Disconnect()){
                cerr << "CRITICAL: Cable initialization failed and rollback is incomplete."
                     << endl;
            }
            return false;
        }
    }
    this->isConnected = true;
    return this->IsConnected();
}

bool Robot::Disconnect()
{
    bool gripperDisconnected = true;
    if (this->gripper.HasActiveResources())
        gripperDisconnected = this->gripper.Disconnect();

    if(this->cable.HasActiveResources()){
        if(!this->cable.StopAllMotor()){
            cerr << "CRITICAL: Cable stop command failed during shutdown. "
                 << "Brakes and controller connections are being retained for "
                 << "a safe retry." << endl;
            this->isConnected = false;
            this->shutdownIncomplete = true;
            return false;
        }
    }

    bool brakesSecured = true;
    if (this->brake.IsConnected()){
        const bool railBrakesClosed = this->brake.CloseAllRailBrake();
        const bool cableBrakesClosed = this->brake.CloseAllCableBrake();
        brakesSecured = railBrakesClosed && cableBrakesClosed;
    }
    if(!brakesSecured){
        cerr << "CRITICAL: One or more brakes could not be closed. "
             << "Motor connections are being retained for a safe retry." << endl;
        this->isConnected = false;
        this->shutdownIncomplete = true;
        return false;
    }

    bool cableDisconnected = true;
    if (this->cable.HasActiveResources())
        cableDisconnected = this->cable.Disconnect();

    bool railDisconnected = true;
    if (this->rail.HasActiveResources())
        railDisconnected = this->rail.Disconnect();

    bool brakeDisconnected = true;
    if (this->brake.HasActiveResources() && cableDisconnected && railDisconnected)
        brakeDisconnected = this->brake.Disconnect();

    this->shutdownIncomplete = !gripperDisconnected
        || !brakeDisconnected
        || this->gripper.HasActiveResources()
        || this->brake.HasActiveResources()
        || this->cable.HasActiveResources()
        || this->rail.HasActiveResources();
    this->isConnected = false;

    if (!this->shutdownIncomplete && this->isOnline)
        this->posLogger.CloseFile();
    if(this->shutdownIncomplete){
        cerr << "CRITICAL: Robot shutdown is incomplete; one or more "
             << "controller connections remain active." << endl;
    }
    return !this->shutdownIncomplete;
}

bool Robot::IsConnected() {
    if(!this->isConnected || this->shutdownIncomplete) return false;
    if(this->useGripper && !this->gripper.IsConnected()) return false;
    if((this->useRailBraker || this->useCableBraker)
        && !this->brake.IsConnected()) return false;
    if(this->useCableMotor && !this->cable.IsConnected()) return false;
    if(this->useRailMotor && !this->rail.IsConnected()) return false;
    return true;
}

bool Robot::IsValid() { return this->isValidModel; }

bool Robot::CheckLimits()
{
    for (int i = 0; i < 3; i++)
    {
        if (this->endEffectorPos[i] < this->endEffectorPosLimit[i * 2])
            return false;
        if (this->endEffectorPos[i] > this->endEffectorPosLimit[i * 2 + 1])
            return false;
    }
    return true;
}

void Robot::PrintEEPos()
{
    cout << "Current EE Pos: " << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->endEffectorPos[i] << "                   " << endl;
    }
}

void Robot::PrintBrickPickUpPos()
{
    cout << "Brick Pick Up Pos: ";
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[0] << "    " << endl;
    }
}

void Robot::PrintRailOffset()
{ // Print railOffset[]
    cout << "Rail Offset: " << endl;
    for (int i = 0; i < this->railMotorNum; i++)
    {
        cout << "\t" << "Cable " << i << ": " << this->railOffset[i] << "    " << endl;
    }
}

void Robot::PrintCableLength()
{
    cout << "Cable Length: ";
    vector<double> cableLength = this->EEPoseToCableLength(this->endEffectorPos);
    for (int i = 0; i < this->cableMotorNum; i++)
    {
        cout << cableLength[i] << " ";
    }
    cout << endl;
    cout << "Rail Offset: ";
    for (int i = 0; i < this->railMotorNum; i++)
    {
        cout << this->railOffset[i] << " ";
    }
    cout << endl;
}

void Robot::PrintHomePos()
{ // Print home[]
    cout << "HOME Pos: ";
    for (int i = 0; i < 6; i++)
    {
        cout << homePos[i] << " ";
    }
    cout << endl;
}

bool Robot::UpdateModelFromFile(string filename, bool reconnect)
{ // Read model.json file
    const bool hasActiveControllers = this->isConnected
        || this->shutdownIncomplete
        || this->gripper.HasActiveResources()
        || this->brake.HasActiveResources()
        || this->cable.HasActiveResources()
        || this->rail.HasActiveResources();
    if(reconnect){
        if(!this->Disconnect()){
            cerr << "Error: Cannot reload configuration while shutdown is incomplete."
                 << endl;
            return false;
        }
    }else if(hasActiveControllers){
        cerr << "Error: Refusing to change configuration while controllers are active. "
             << "Use reconnect mode." << endl;
        return false;
    }
    json model;
    try
    {
        ifstream file(filename, ifstream::binary);
        if (!file.good())
        {
            cout << "Error: File Not Exist. Keep using current config." << endl;
            return false;
        }
        file >> model;
        cout << "Importing " << filename << ": " << model.value("ModelName", "NOT FOUND") << endl;

        this->isOnline = model.value("isOnline", false);

        // Initialize parameters
        const int configuredCableMotorNum = model.value("cableMotorNum", 8);
        const int configuredRailMotorNum = model.value("railMotorNum", 4);
        if(configuredCableMotorNum < 1 || configuredCableMotorNum > 8){
            throw runtime_error("cableMotorNum must be between 1 and 8");
        }
        if(configuredRailMotorNum < 1 || configuredRailMotorNum > 4){
            throw runtime_error("railMotorNum must be between 1 and 4");
        }
        this->cableMotorNum = configuredCableMotorNum;
        this->railMotorNum = configuredRailMotorNum;
        this->railMotorCableMotorOffset = model.value("railMotorCableMotorOffset", 2);
        this->absTrqLmt = model.value("absTrqLmt", 10);
        this->endEffToGroundOffset = model.value("endEffToGroundOffset", -0.28);
        this->cameraOffset[0] = model.value("cameraOffset.x", -0.2383);
        this->cameraOffset[1] = model.value("cameraOffset.y", -0.09666);
        this->targetTrq = model.value("targetTorque", -2.5);
        this->cableMotorScale = model.value("cableMotorScale", 509295);
        this->railMotorScale = model.value("railMotorScale", 38400000);
        this->pulleyRadius = model.value("pulleyRadius", 0.045);
        this->rotationalAngleOffset = model.value("rotationalAngleOffset", 21.8);
        this->rotationalDistanceOffset = model.value("rotationalDistanceOffset", -0.0075);
        this->gripperCommPort = model.value("gripperCommPort", "\\\\.\\COM26");
        this->railBrakeCommPort = model.value("railBrakeCommPort", "\\\\.\\COM26");
        this->brickPickUpOffset = model.value("brickPickUpOffset", 0.12);
        this->endEffVelLmt = model.value("endEffVelLmt", 0.45);
        this->railVelLmt = model.value("railVelLmt", 0.01);
        this->useIndividualCableScale = model.value("useIndividualCableScale", false);
        this->useGripper = model.value("useGripper", false);
        this->useCableMotor = model.value("useCableMotor", false);
        this->useCableBraker = model.value("useCableBraker", false);
        this->useRailMotor = model.value("useRailMotor", false);
        this->useRailBraker = model.value("useRailBraker", false);
        this->cable.SetTrqLmt(this->absTrqLmt);
        this->MILLIS_TO_NEXT_FRAME = model.value("controlFreq", 50);
        this->RPiIP = model.value("RPiIP", "192.168.1.100");
        this->RPiPort = model.value("RPiPort", 8080);
        string posLabel[] = {"x", "y", "z", "roll", "pitch", "yaw"};

        // Interate through arrays
        for (int i = 0; i < 6; i++)
        {
            this->homePos[i] = model["homePos"][posLabel[i]];
            this->endEffectorPosLimit[i] = (float)model["endEffectorPosLimit"][posLabel[i]];
            this->brickPickUpPos[i] = model["brickPickUpPos"][posLabel[i]];
        }
        for (int i = 0; i < cableMotorNum; i++)
        {
            frmOutUnitV[i] << 0, 0, model["pulleyZdir"][i];
            for (int j = 0; j < 3; j++)
            { // for each xyz coordinates
                this->frmOut[i][j] = model["frameAttachments"][i][j];
                this->endOut[i][j] = model["endEffectorAttachments"][i][j];
            }
        }
        if (this->useIndividualCableScale)
        {
            if (model.contains("cableMotorScaleIndividual"))
            {
                for (int i = 0; i < this->cableMotorNum; i++)
                {
                    this->cableMotorScaleIndividual[i] = (float)model["cableMotorScaleIndividual"][i];
                }
            }
        }
        else
        {
            for (int i = 0; i < this->cableMotorNum; i++)
            {
                this->cableMotorScaleIndividual[i] = this->cableMotorScale;
            }
        }
        if (!this->isOnline)
            cout << "!!!!!Warning!!!!!Offline Mode. Modify the config file if you want go online" << endl;
        this->isValidModel = true;
        if (reconnect && !this->Connect()){
            cerr << "Error: Configuration was loaded, but robot reconnection failed."
                 << endl;
            return false;
        }
        return true;
    }
    catch (const exception &e)
    {
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
        cout << "Error: Cannot read robot config file. Exit..." << endl;
        this->isValidModel = false;
        return false;
    }
}

bool Robot::UpdatePos(double pos[6]){
    if(this->isOnline && !this->IsConnected()){
        cerr << "Position calibration rejected: robot is not fully connected."
             << endl;
        return false;
    }
    for(int i = 0; i < 6; i++){
        this->endEffectorPos[i] = pos[i];
    }
    
    vector<double> cableLengthList = this->GetCableLength();
    // calibration motors
    if(this->isOnline){
        // cable motors
        cout << "Calibrating Cable Motor" << endl;
        for(int index = 0; index < this->cableMotorNum; index++){
            if(!this->cable.CalibrationMotor(
                index,
                this->CableMotorLengthToCmd(index, cableLengthList[index])
            )){
                cerr << "Cable calibration failed at motor " << index
                     << "; remaining calibration writes were not issued."
                     << endl;
                this->isConnected = false;
                this->shutdownIncomplete = true;
                return false;
            }
        }
    }
    this->PrintEEPos();
    // for teknic motors
    if(this->useCableMotor){
        cout << "Cable Motor length / internal counts: " << endl;
        for (int i = 0; i < this->cableMotorNum; i++){
            cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i)  << "    " << endl;
        }

    }
    return true;
}

bool Robot::UpdatePosFromFile(string filename, bool calibration){
    if(this->isOnline
        && calibration
        && !this->IsConnected()){
        cerr << "Position-file calibration rejected: robot is not fully connected."
             << endl;
        return false;
    }
    try{
        ifstream file (filename); //"lastPos.txt" or "currentPos.csv"
        if(!file.good()){
            cout << "Error: File Not Exist. Keep using current pos." << endl;
            return false;
        }
        string temp;
        int count = 0;
        if (file.is_open())
        {
            try
            {
                while (file >> temp) // read the file cell by cell
                {
                    if (count < 6)
                    {
                        this->endEffectorPos[count] = stod(temp);
                    } // convert string to double stod() for the first 6 inputs
                    else if (count < 10)
                    {
                        this->railOffset[count - 6] = stod(temp);
                        this->startRailOffset[count - 6] = this->railOffset[count - 6];
                    } // reading the rail offset, then break while loop
                    else
                    {
                        break;
                    }
                    count++;
                }
                cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
            }
            catch (int e)
            {
                cout << "Check if currentPos.csv matches the in1 input no." << endl;
            }
            vector<double> cableLengthList = this->GetCableLength();
            // calibration motors
            if (this->isOnline && calibration)
            {
                // cable motors
                cout << "Calibrating Cable Motor" << endl;
                for (int index = 0; index < this->cableMotorNum; index++)
                {
                    if(!this->cable.CalibrationMotor(
                        index,
                        this->CableMotorLengthToCmd(index, cableLengthList[index])
                    )){
                        cerr << "Cable calibration from position file failed at "
                             << "motor " << index << "." << endl;
                        this->isConnected = false;
                        this->shutdownIncomplete = true;
                        return false;
                    }
                }
                // slider motors
                cout << "Calibrating Rail Motor" << endl;
                for(int index = 0; index < this->railMotorNum; index++){
                    if(!this->rail.CalibrationMotor(
                        index,
                        this->RailMotorLengthToCmd(index, railOffset[index])
                    )){
                        cerr << "Rail calibration from position file failed at "
                             << "motor " << index << "." << endl;
                        this->isConnected = false;
                        this->shutdownIncomplete = true;
                        return false;
                    }
                }
                cout << "Updating motor counts completed" << endl;
            }
            this->PrintEEPos();
            // for teknic motors
            if (this->useCableMotor)
            {
                cout << "Cable Motor length / internal counts: " << endl;
                for (int i = 0; i < this->cableMotorNum; i++)
                {
                    cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i) << "    " << endl;
                }
            }
            // for twincat motors
            if (this->useRailMotor)
            {
                cout << "Rail Motor length / internal counts: " << endl;
                vector<double> railMotorCmd;
                if(!this->rail.GetMotorPosMeasured(railMotorCmd)){
                    cerr << "Failed to verify rail positions after calibration."
                         << endl;
                    this->isConnected = false;
                    this->shutdownIncomplete = true;
                    return false;
                }
                for (int i = 0; i < this->railMotorNum; i++)
                {
                    const double measuredPosition =
                        static_cast<size_t>(i) < railMotorCmd.size() ? railMotorCmd[i] : 0.0;
                    cout << "\t" << "Rail " << i << ": " << this->railOffset[i]
                         << " / " << measuredPosition << "    " << endl;
                }
            }

            if (this->isOnline)
                this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
            return true;
        }
        return false;
    }
    catch (const exception &e)
    {
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
        return false;
    }
}

bool Robot::UpdatePosFromLog(string filename)
{
    if(this->isOnline && !this->IsConnected()){
        cerr << "Position-log calibration rejected: robot is not fully connected."
             << endl;
        return false;
    }
    try
    {
        ifstream file(filename); //"lastPos.txt" or "currentPos.csv"
        if (!file.good())
        {
            cout << "Error: File Not Exist. Keep using current pos." << endl;
            return false;
        }
        string temp;
        int count = 0;
        if (file.is_open())
        {
            try
            {
                while (file >> temp)
                {
                    if (count < 6)
                    {
                        this->endEffectorPos[count] = stod(temp);
                    } // convert string to double stod() for the first 6 inputs
                    else if (count < 10)
                    {
                        this->railOffset[count - 6] = stod(temp);
                    } // reading the rail offset, then break while loop
                    else
                    {
                        break;
                    }
                    count++;
                }
                cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
            }
            catch (int e)
            {
                cout << "Check if currentPos.csv matches the in1 input no." << endl;
            }
            vector<double> cableLengthList = this->GetCableLength();
            // calibration motors
            if (this->isOnline)
            {
                // cable motors
                cout << "Calibrating Cable Motor" << endl;
                for (int index = 0; index < this->cableMotorNum; index++)
                {
                    if(!this->cable.CalibrationMotor(
                        index,
                        this->CableMotorLengthToCmd(index, cableLengthList[index])
                    )){
                        cerr << "Cable calibration from position log failed at "
                             << "motor " << index << "." << endl;
                        this->isConnected = false;
                        this->shutdownIncomplete = true;
                        return false;
                    }
                }
                // slider motors
                cout << "Calibrating Rail Motor" << endl;
                for (int index = 0; index < this->railMotorNum; index++)
                {
                    if(!this->rail.CalibrationMotor(
                        index,
                        this->RailMotorLengthToCmd(index, railOffset[index])
                    )){
                        cerr << "Rail calibration from position log failed at "
                             << "motor " << index << "." << endl;
                        this->isConnected = false;
                        this->shutdownIncomplete = true;
                        return false;
                    }
                }
                cout << "Updating motor counts completed" << endl;
            }
            this->PrintEEPos();
            // for teknic motors
            cout << "Cable Motor length / internal counts: " << endl;
            for (int i = 0; i < this->cableMotorNum; i++)
            {
                cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i) << "    " << endl;
            }
            // for twincat motors
            cout << "Rail Motor length / internal counts: " << endl;
            vector<double> railMotorCmd;
            if(this->useRailMotor
                && !this->rail.GetMotorPosMeasured(railMotorCmd)){
                cerr << "Failed to verify rail positions after log recovery."
                     << endl;
                this->isConnected = false;
                this->shutdownIncomplete = true;
                return false;
            }
            for (int i = 0; i < this->railMotorNum; i++)
            {
                const double measuredPosition =
                    static_cast<size_t>(i) < railMotorCmd.size() ? railMotorCmd[i] : 0.0;
                cout << "\t" << "Rail " << i << ": " << this->railOffset[i]
                     << " / " << measuredPosition << "    " << endl;
            }

            if (this->isOnline)
                this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
            return true;
        }
        return false;
    }
    catch (const exception &e)
    {
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
        return false;
    }
}

void Robot::PrintRobotConfig()
{

    // print to screen
    cout << "Cable Motor Number:\t\t" << this->cableMotorNum << endl;
    cout << "Rail Motor Number:\t\t" << this->railMotorNum << endl;
    cout << "End-Effector to Ground Offset:\t" << this->endEffToGroundOffset << endl;
    cout << "Absolute Torque Limit\t\t" << this->absTrqLmt << endl;
    cout << "Target Torque\t\t\t" << this->targetTrq << endl;
    cout << "Cable Motor Scale\t\t" << this->cableMotorScale << endl;
    cout << "Rail Motor Scale\t\t" << this->railMotorScale << endl;
    cout << "Pully Radius\t\t\t" << this->pulleyRadius << endl;
    cout << "Rotational Angle Offset\t\t" << this->rotationalAngleOffset << endl;
    cout << "Rotational Distance Offset\t" << this->rotationalDistanceOffset << endl;
    cout << "Gripper Comm Port\t\t" << this->gripperCommPort << endl;
    cout << "Rail Brake Comm Port\t\t" << this->railBrakeCommPort << endl;

    cout << "End Effector Pos Limit: " << endl;
    for (int i = 0; i < 3; i++)
    {
        cout << "\t" << posLabel[i] << ": (" << this->endEffectorPosLimit[i * 2] << " ~ " << this->endEffectorPosLimit[i * 2 + 1] << ")" << endl;
    }

    cout << "Home Pos: " << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->homePos[i] << endl;
    }

    cout << "Brick Pick Up Pos: " << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[i] << endl;
    }
}

vector<double> Robot::EEPoseToCableLength(double eePos[], double railOffset[])
{ // given an EE pose, calculate the EE to pulley cable lengths
    cu_brick::CableKinematicsConfig config;
    config.cableCount = static_cast<size_t>(this->cableMotorNum);
    config.pulleyRadius = this->pulleyRadius;
    for(size_t index = 0; index < config.cableCount; index++){
        config.frameAttachments[index] = this->frmOut[index];
        config.endEffectorAttachments[index] = this->endOut[index];
        config.pulleyAxes[index] = this->frmOutUnitV[index];
    }
    for(size_t index = 0; index < cu_brick::kMaximumRailCount; index++)
        config.startingRailOffsets[index] = this->startRailOffset[index];

    array<double, 6> pose;
    copy(eePos, eePos + pose.size(), pose.begin());
    array<double, cu_brick::kMaximumRailCount> currentRailOffsets;
    copy(
        railOffset,
        railOffset + currentRailOffsets.size(),
        currentRailOffsets.begin()
    );
    return cu_brick::ComputeCableLengths(
        config,
        pose,
        currentRailOffsets
    );
}

vector<double> Robot::EEPoseToCableLength(double eePos[])
{
    return this->EEPoseToCableLength(eePos, this->railOffset);
}

vector<double> Robot::EEPoseToCableLength(vector<double> eePosVector)
{
    return this->EEPoseToCableLength(eePosVector, this->railOffset);
}

vector<double> Robot::EEPoseToCableLength(vector<double> eePosVector, double railOffset[])
{
    double *eePosArray = &eePosVector[0];
    return this->EEPoseToCableLength(eePosArray, railOffset);
}

double Robot::GetEEToGroundOffset() { return endEffToGroundOffset; }

double* Robot::GetCameraOffset() { return cameraOffset; }

float Robot::GetWorkingTrq() { return targetTrq; }

float Robot::GetAbsTrqLmt() { return absTrqLmt; }

int Robot::GetCableMotorNum() { return cableMotorNum; }

int Robot::GetRailMotorNum() { return railMotorNum; }

string Robot::GetGripperCommPort() { return gripperCommPort; }

string Robot::GetRailBrakeCommPort() { return railBrakeCommPort; }

string Robot::GetCableBrakeCommPort() { return cableBrakeCommPort; }

double Robot::GetCableMotorScale(int motorID) { return cableMotorScaleIndividual[motorID]; }

double Robot::GetRailMotorScale() { return railMotorScale; }

float Robot::GetEffVelLmt() { return endEffVelLmt; }

int Robot::GetCableMotorBrakeNum() { return cableMotorBrakeNum; }

bool Robot::GetMoveToBrickPos(SOCKET udpSocket, sockaddr_in serverAddr, double offsets[], double& angle) {

    // Check if socket is valid
    if (udpSocket == INVALID_SOCKET) {
        cout << "Error: Invalid socket" << endl;
        system("pause");
        return false;
    }

    // Set socket timeout (5 seconds)
    DWORD timeout = 5000; // 5 seconds in milliseconds
    if (setsockopt(udpSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) == SOCKET_ERROR) {
        cout << "Warning: Failed to set socket timeout" << endl;
    }

    // Send request
    const char* request = "get_brick_position";
    if (sendto(udpSocket, request, strlen(request), 0, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        cout << "Failed to send request" << endl;
        system("pause");
        return false;
    }

    // Receive response
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer)); // Initialize buffer
    int serverAddrLen = sizeof(serverAddr);
    
    int bytesReceived = recvfrom(udpSocket, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&serverAddr, &serverAddrLen);
    
    if (bytesReceived == SOCKET_ERROR) {
        cout << "Failed to receive response" << endl;
        system("pause");
        return false;
    }

    // Null terminate the received data
    buffer[bytesReceived] = '\0';
    json jsonData;
    try{
        jsonData = json::parse(buffer);
    }catch(const json::parse_error& e){
        cout << "Error parsing JSON: " << e.what() << endl;
        cout << "Received data: " << buffer << endl;
        system("pause");
        return false;
    }

    // Validate JSON structure
    if (!jsonData.contains("x") || !jsonData.contains("y") || !jsonData.contains("z") || !jsonData.contains("angle")) {
        cout << "Error: Missing required coordinates in JSON response" << endl;
        cout << "Received JSON: " << jsonData.dump() << endl;
        system("pause");
        return false;
    }

    try {
        offsets[1] = jsonData["x"].get<double>();
        offsets[0] = jsonData["y"].get<double>();
        offsets[2] = 0.5 - jsonData["z"].get<double>();
        angle = jsonData["angle"].get<double>();

        cout << "Brick position offsets: x=" << offsets[1] << ", y=" << offsets[0] << ", z=" << offsets[2] << ", angle=" << angle << endl;
    } catch (const exception& e) {
        cout << "Error extracting coordinates from JSON: " << e.what() << endl;
        system("pause");
        return false;
    }

    return true;
}


int32_t Robot::CableMotorLengthToCmd(int motorID, double length)
{
    return length * cableMotorScaleIndividual[motorID];
}

double Robot::CableMotorCmdToLength(int motorID, int cmd)
{
    return cmd / cableMotorScaleIndividual[motorID];
}

int32_t Robot::RailMotorLengthToCmd(int motorID, double length)
{
    return length * railMotorScale;
}

double Robot::RailMotorCmdToLength(int motorID, int cmd)
{
    return cmd / railMotorScale;
}

vector<double> Robot::GetCableLength()
{
    return this->EEPoseToCableLength(this->endEffectorPos);
};

vector<int32_t> Robot::EEPoseToCmd(vector<double> eePos)
{
    return EEPoseToCmd(eePos, this->railOffset);
}

vector<int32_t> Robot::EEPoseToCmd(vector<double> eePosVector, double railOffset[])
{
    double *eePosArray = &eePosVector[0];
    return this->EEPoseToCmd(eePosArray, railOffset);
}

vector<int32_t> Robot::EEPoseToCmd(double eePos[])
{
    return EEPoseToCmd(eePos, this->railOffset);
}

vector<int32_t> Robot::EEPoseToCmd(double eePos[], double railOffset[])
{
    vector<int32_t> result;
    vector<double> lengthList = this->EEPoseToCableLength(eePos, railOffset);
    int index = 0;
    for (double length : lengthList)
    {
        // result.push_back(this->CableMotorLengthToCmd(index, length));
        result.push_back(this->CableMotorLengthToCmd(index, length));
        index++;
    }
    return result;
};

bool Robot::eBrake(bool cableBrake, bool railBrake)
{
    if (!useEBrake)
        return true;
    if (kbhit())
    { // Emergency quit during trajectory control
        const bool cableStopped = this->cable.StopAllMotor();
        if(!cableStopped){
            cerr << "CRITICAL: Emergency stop could not verify cable motor stop. "
                 << "Brakes were not changed; controller state is retained for "
                 << "a safe retry." << endl;
            this->isConnected = false;
            this->shutdownIncomplete = true;
            return false;
        }
        bool shutdownCommandsSucceeded = true;
        if (cableBrake)
            shutdownCommandsSucceeded =
                this->brake.CloseAllCableBrake() && shutdownCommandsSucceeded;
        if (railBrake)
            shutdownCommandsSucceeded =
                this->brake.CloseAllRailBrake() && shutdownCommandsSucceeded;
        if(!shutdownCommandsSucceeded){
            cerr << "CRITICAL: Emergency stop could not verify all stop/brake "
                 << "commands." << endl;
            this->isConnected = false;
            this->shutdownIncomplete = true;
        }
        string userInput;
        cout << endl;
        cout << "================================== Trajectory Stopped!! ==================================" << endl;
        system("pause");
        // An emergency stop ends the operational session. Active controller
        // resources remain tracked so the main loop can perform verified
        // shutdown instead of resuming motion.
        this->isConnected = false;
        this->shutdownIncomplete = true;
        return false;
    }
    return true;
};

vector<vector<int32_t>> Robot::PoseTrajToCmdTraj(vector<vector<double>> posTraj)
{
    vector<vector<int32_t>> result;
    for (vector<double> pos : posTraj)
    {
        result.push_back(this->EEPoseToCmd(pos));
    }
    return result;
}

bool Robot::RunCableTraj(vector<vector<double>> posTraj, bool showAtten)
{
    if(!this->IsConnected()){
        cerr << "Trajectory rejected: robot is not fully connected." << endl;
        return false;
    }
    if(posTraj.empty()){
        cerr << "Trajectory rejected: no position frames were provided." << endl;
        return false;
    }
    for(size_t frameIndex = 0; frameIndex < posTraj.size(); frameIndex++){
        if(posTraj[frameIndex].size() < 6){
            cerr << "Trajectory rejected: position frame " << frameIndex
                 << " has fewer than six pose values." << endl;
            return false;
        }
    }

    vector<vector<int32_t>> cmdTraj = this->PoseTrajToCmdTraj(posTraj);
    const size_t cableCount = static_cast<size_t>(this->cableMotorNum);
    if(cmdTraj.size() != posTraj.size()){
        cerr << "Trajectory rejected: command frame generation was incomplete."
             << endl;
        return false;
    }
    for(size_t frameIndex = 0; frameIndex < cmdTraj.size(); frameIndex++){
        if(cmdTraj[frameIndex].size() < cableCount){
            cerr << "Trajectory rejected: command frame " << frameIndex
                 << " has fewer values than the configured cable count."
                 << endl;
            return false;
        }
        cmdTraj[frameIndex].resize(cableCount);
    }

    size_t index = 0;
    if (showAtten)
    {
        for (int i = 0; i < 18; i++)
        {
            cout << endl;
        }
    }
    // check every step to make sure the command is not too large
    vector<int32_t> lastFrameCmd(cableCount, 0);
    vector<double> lastFrameLength = this->EEPoseToCableLength(this->endEffectorPos);
    if(lastFrameLength.size() < cableCount){
        cerr << "Trajectory rejected: current cable-length calculation is "
             << "smaller than the configured cable count." << endl;
        return false;
    }
    for (size_t i = 0; i < cableCount; i++)
    {
        lastFrameCmd[i] = lastFrameLength[i] * this->cableMotorScaleIndividual[i];
    }
    cout << "Last Frame Length:";
    for(size_t i = 0; i < cableCount; i++)
        cout << " " << lastFrameLength[i];
    cout << endl;
    cout << "Last Frame Cmd:";
    for(size_t i = 0; i < cableCount; i++)
        cout << " " << lastFrameCmd[i];
    cout << endl;
    vector<double> lastFrameEEPos = {this->endEffectorPos[0], this->endEffectorPos[1], this->endEffectorPos[2]};
    for (size_t j = 0; j < posTraj.size(); j++)
    {
        vector<int32_t> frame = cmdTraj[j];
        for (size_t i = 0; i < cableCount; i++)
        {
            if ((frame[i] - lastFrameCmd[i]) / this->cableMotorScaleIndividual[i] > 0.5 || (frame[i] - lastFrameCmd[i]) / this->cableMotorScaleIndividual[i] < -0.5)
            {
                cout << "================================== Trajectory Stopped!! ==================================" << endl;
                cout << "Motor " << i << endl;
                cout << "Frame " << index << endl;
                cout << "Step too large! range should be in -0.5m(" << -0.5 * this->cableMotorScaleIndividual[i] << ") ~ 0.5m(" << 0.5 * this->cableMotorScaleIndividual[i] << ")." << endl;
                cout << "Current command: " << frame[i] << " Length: " << frame[i] / this->cableMotorScaleIndividual[i] << endl;
                cout << "Last frame command: " << lastFrameCmd[i] << " Length: " << lastFrameCmd[i] / this->cableMotorScaleIndividual[i] << endl;
                cout << "Difference: " << (frame[i] - lastFrameCmd[i]) / this->cableMotorScaleIndividual[i] << "m" << endl;
                cout << "Last frame EE Pos: x" << lastFrameEEPos[0] << " y" << lastFrameEEPos[1] << " z" << lastFrameEEPos[2] << endl;
                cout << "Cmd EE Pos: x" << posTraj[index][0] << " y" << posTraj[index][1] << " z" << posTraj[index][2] << endl;
                return false;
            }
            lastFrameCmd[i] = frame[i];
        }
        lastFrameEEPos[0] = posTraj[j][0];
        lastFrameEEPos[1] = posTraj[j][1];
        lastFrameEEPos[2] = posTraj[j][2];
    }

    for (vector<int32_t> frame : cmdTraj)
    {
        auto start = chrono::steady_clock::now();
        if (!this->eBrake(true, false))
            return false;
        if (!this->cable.MoveAllMotorCmd(frame))
        {
            cout << "Trajectory stopped" << endl;
            // At least one command write may have reached its motor. Even if
            // the controller subsequently verifies a stop, the software pose
            // can no longer be treated as authoritative.
            this->isConnected = false;
            this->shutdownIncomplete = true;
            return false;
        }
        for (int i = 0; i < 6; i++)
            this->endEffectorPos[i] = posTraj[index][i];
        index++;
        if (this->isOnline)
            this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
        if (showAtten)
        {
            printf("\x1b[18A");
            cout << "Current frame: " << index << endl;
            this->PrintEEPos();
            cout << "Current Cable Motor Count: " << endl;
            for (int i = 0; i < this->cableMotorNum; i++)
            {
                cout << "\tCable " << i << ": " << frame[i] << endl;
            }
            // cout << "Current Measured Cable Motor Trq: " << endl;
            // for (int i = 0; i < this->cableMotorNum; i++){
            //     cout << "\tCable " << i << ": " << this->cable.GetMotorTorqueMeasured(i) << endl;
            // }
        }
        auto end = chrono::steady_clock::now();
        double dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end - start).count() - 1;
        cout << "dif: " << dif << endl;
        while (dif > 0)
        {
            end = chrono::steady_clock::now();
            dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end - start).count() - 1;
        }
    }
    return true;
}

bool Robot::MoveToLinear(double dest[], int time, bool showAtten, bool useEBrake)
{
    if (!useEBrake)
        this->useEBrake = false;
    bool result = this->RunCableTraj(GenLinearTrajForCableMotor(this->endEffectorPos, dest, time, showAtten), showAtten);
    this->useEBrake = true;
    return result;
}

bool Robot::MoveToParaBlend(double dest[], int time, bool showAtten)
{
    return this->RunCableTraj(GenParaBlendTrajForCableMotor(this->endEffectorPos, dest, time), showAtten);
}

bool Robot::MoveToParaBlend(double dest[], bool showAtten)
{
    float time = sqrt(pow(dest[0] - this->endEffectorPos[0], 2) + pow(dest[1] - endEffectorPos[1], 2) + pow(dest[2] - endEffectorPos[2], 2)) / this->endEffVelLmt * 1000;
    time = max(time, 1000.0); // ensure minimum time is 1 second
    return this->RunCableTraj(GenParaBlendTrajForCableMotor(this->endEffectorPos, dest, time), showAtten);
}


bool Robot::RaiseRailWithCableByLengthAbsulote(int railIndex, int cableIndex, float target)
{
    if(!this->IsConnected()){
        cerr << "Coordinated rail move rejected: robot is not fully connected."
             << endl;
        return false;
    }
    const auto markUnsafeShutdownState = [this](){
        this->isConnected = false;
        this->shutdownIncomplete = true;
    };
    if(railIndex < 0 || railIndex >= this->railMotorNum
        || cableIndex < 0 || cableIndex >= this->cableMotorNum){
        cerr << "Invalid rail/cable index for coordinated move." << endl;
        return false;
    }
    if(!std::isfinite(target)){
        cerr << "Rail target must be finite." << endl;
        return false;
    }
    if (target < 0 || target > 2.55)
    {
        cout << "WARNING! Intended rail count is out of bound! " << target << endl;
        cout << "Valid range: " << 0 << " ~ " << 2.55 << endl;
        if(!CheckContinue()){
                return false;
        }
    }

    const bool cableBrakeOpened =
        this->brake.OpenCableBrakeByIndex(cableIndex);
    const bool railBrakeOpened =
        this->brake.OpenRailBrakeByIndex(railIndex);
    if(!cableBrakeOpened || !railBrakeOpened){
        cerr << "Coordinated move aborted because a brake could not be opened."
             << endl;
        bool rollbackSucceeded = true;
        if(cableBrakeOpened)
            rollbackSucceeded =
                this->brake.CloseCableBrakeByIndex(cableIndex)
                && rollbackSucceeded;
        if(railBrakeOpened)
            rollbackSucceeded =
                this->brake.CloseRailBrakeByIndex(railIndex)
                && rollbackSucceeded;
        // A failed write is indeterminate: the brake may have moved even
        // though the command was not acknowledged. End the operational
        // session regardless of whether compensating closes were acknowledged.
        if(!rollbackSucceeded)
            cerr << "CRITICAL: Brake-open rollback could not be verified."
                 << endl;
        markUnsafeShutdownState();
        return false;
    }

    const auto secureBrakes = [this, railIndex, cableIndex](){
        const bool cableClosed =
            this->brake.CloseCableBrakeByIndex(cableIndex);
        const bool railClosed =
            this->brake.CloseRailBrakeByIndex(railIndex);
        if(!cableClosed || !railClosed){
            cerr << "CRITICAL: Coordinated move ended with an unsecured brake."
                 << endl;
            this->isConnected = false;
            this->shutdownIncomplete = true;
        }
        return cableClosed && railClosed;
    };

    double dura = abs(target - this->railOffset[railIndex]) / this->railVelLmt * 1000; // *1000 to change unit to ms
    if(!std::isfinite(dura) || dura <= 0){
        cerr << "Invalid coordinated move duration." << endl;
        secureBrakes();
        return false;
    }
    if (dura <= 200 || dura > 10000)
    {
        cout << "Invalid duration: " << dura << " ms" << endl;
        if(!CheckContinue()){
            secureBrakes();
            return false;
        }
    } // Don't run traj for incorrect timing

    static double a, b, c; // coefficients for cubic spline trajectory
    cout << "ATTENTION: Raise rail function is called. Estimated time: " << dura / 1000 << " seconds" << endl;
    cout << "Generating raise rail trajectory..." << endl;

    // Solve coefficients of equations for cubic
    a = this->railOffset[railIndex];
    b = 3 / (dura * dura) * (target - this->railOffset[railIndex]);
    c = -2 / (dura * dura * dura) * (target - this->railOffset[railIndex]);

    // Generate rail and cable traj.
    long nErr;
    double step{};
    double t = 0;
    long dur = 0;
    double railOffset[6] = {0};
    vector<double> cableLength;
    vector<int32_t> cableTraj;
    vector<int32_t> railTraj;
    while (t <= dura)
    {
        railOffset[railIndex] = a + b * t * t + c * t * t * t; // update new rail offset
        // cout << "Rail offset: " << railOffset[railIndex] << endl;
        int32_t cableFrame = this->EEPoseToCmd(this->endEffectorPos, railOffset)[cableIndex];
        cableTraj.push_back(cableFrame);
        int32_t railFrame = this->RailMotorLengthToCmd(railIndex, railOffset[railIndex]);
        railTraj.push_back(railFrame);
        t += MILLIS_TO_NEXT_FRAME;
    }

    cout << "Raise rail trajectory Generated" << endl;
    for (int i = 0; i < cableTraj.size(); i++)
    {
        if (!this->eBrake(true, true))
        {
            return false;
        }
        auto start = chrono::steady_clock::now();
        if(!rail.MoveMotorCmdAbsuloteById(railIndex, railTraj[i])){
            cerr << "Move rail " << railIndex << " failed!" << endl;
            if(!this->cable.StopAllMotor()){
                markUnsafeShutdownState();
                return false;
            }
            secureBrakes();
            // A rejected acknowledgement cannot prove whether the rail moved.
            // End the session instead of continuing from an uncertain pose.
            markUnsafeShutdownState();
            return false;
        }
        cout << "Frame " << i << " Rail " << railIndex << " Cmd: " << railTraj[i] << "(" << this->RailMotorCmdToLength(railIndex, railTraj[i]) << "m)" 
                              << " Cable " << cableIndex << " Cmd: " << cableTraj[i] << "(" << this->CableMotorCmdToLength(cableIndex, cableTraj[i]) << "m)" << endl;
        if (!cable.MoveSingleMotorCmd(cableIndex, cableTraj[i], true))
        {
            cout << "Move cable " << cableIndex << " failed!";
            if(!this->cable.StopAllMotor()){
                markUnsafeShutdownState();
                return false;
            }
            secureBrakes();
            // The rail command for this frame may already have executed while
            // the cable write failed, so the coordinated pose is indeterminate.
            markUnsafeShutdownState();
            return false;
        }
        this->railOffset[railIndex] = this->RailMotorCmdToLength(railIndex, railTraj[i]);
        if (this->isOnline)
            this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
        dur = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
        if (dif > 0)
        {
            Sleep(dif);
        }
    }

    // Wait for rail motor to reach intented position before enabling brake again
    // while(posNow[railIndex]>step+2 || posNow[railIndex]<step-2){
    //     nErr = AdsSyncReadReq(pAddr, ADSIGRP_SYM_VALBYHND, hdlList[2], sizeof(posNow), &posNow[0]); // read "MAIN.AxisList[railIndex].NcToPlc.ActPos"
    //     if (nErr) { cout << "Error: Rail[" << id << "] AdsSyncReadReq: " << nErr << '\n'; break; }
    //     // cout << posNow[railIndex] << endl;
    // }
    Sleep(50);
    cout << "Rail traj done .... \n";

    return secureBrakes();
}

void Robot::SavePosToFile(string filename)
{
    if (!this->isOnline)
        return;

    //// Safe system shut down, safe last pos and emegency shut down
    // Saving last position before quiting programme
    cout << "Saving last position to file: " << filename << endl;
    ofstream myfile;
    myfile.open(filename);
    for (int i = 0; i < 6; i++)
    {
        myfile << this->endEffectorPos[i] << " ";
    }
    myfile << endl;

    for (int i = 0; i < this->railMotorNum; i++)
    {
        myfile << this->railOffset[i] << " ";
    }
    myfile << endl;

    for (int i = 0; i < this->cableMotorNum; i++)
    {
        myfile << this->cable.GetMotorPosMeasured(i) << " ";
    }
    vector<double> railMotorCmd = this->rail.GetMotorPosMeasured();
    for (int i = 0; i < this->railMotorNum; i++)
    {
        if (this->useRailMotor && static_cast<size_t>(i) < railMotorCmd.size())
        {
            myfile << railMotorCmd[i] << " ";
        }
        else
        {
            myfile << 0 << " ";
        }
    }

    myfile << endl;
}
