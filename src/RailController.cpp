#include "../include/RailController.h"
#include <cmath>
#include <iostream>

RailController::RailController(bool isOnline, bool useRail)
    : motorNode(isOnline), isOnline(isOnline), useRail(useRail) {}

bool RailController::IsValidRailIndex(int index) const {
    return index >= 0 && index < this->railNumber;
}

int RailController::ToPhysicalRailIndex(int logicalIndex) const {
    // Preserve the installation-specific logical-to-physical mapping.
    return (logicalIndex + 2) % 4;
}

bool RailController::Connect(int motorPortNumber, int railNumber){
    if(this->HasActiveResources()){
        if(!this->Disconnect()){
            cerr << "Existing rail connection could not be safely closed." << endl;
            return false;
        }
    }
    this->isConnected = false;
    this->cleanupPending = false;
    if(!this->useRail) return true;
    if(railNumber <= 0 || railNumber > 4){
        cout << "Failed to connect rail motors: rail count must be between 1 and 4."
             << endl;
        return false;
    }

    this->railNumber = railNumber;
    this->workingMotor = this->ToPhysicalRailIndex(0);
    if(!this->motorNode.Connect(motorPortNumber)){
        cout << "Failed to connect rail motors. Exit programme." << endl;
        this->cleanupPending = this->motorNode.IsConnected();
        return false;
    }
    cout << "Rail Controller Online." << endl;
    this->isConnected = true;
    this->cleanupPending = false;
    return true;
}

bool RailController::Disconnect(){
    if(!this->HasActiveResources() && !this->motorNode.IsConnected()) return true;
    if(!this->motorNode.Disconnect()){
        cerr << "Failed to safely disconnect rail motors." << endl;
        this->isConnected = false;
        this->cleanupPending = true;
        return false;
    }
    cout << "Rail Controller Offline." << endl;
    this->isConnected = false;
    this->cleanupPending = false;
    return true;
}

bool RailController::IsConnected() const {
    return this->isConnected;
}

bool RailController::HasActiveResources() const {
    return this->isConnected || this->cleanupPending;
}

bool RailController::SelectWorkingMotor(int index){
    if(!this->useRail) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidRailIndex(index)){
        cerr << "Invalid rail motor index: " << index << endl;
        return false;
    }
    this->workingMotor = this->ToPhysicalRailIndex(index);
    return true;
}

bool RailController::MoveMotorCmdAbsuloteById(int id, double cmd){
    if(!this->useRail) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidRailIndex(id)){
        cerr << "Invalid rail motor index: " << id << endl;
        return false;
    }
    if(!std::isfinite(cmd)){
        cerr << "Rail command must be finite." << endl;
        return false;
    }

    const int physicalId = this->ToPhysicalRailIndex(id);
    if(!this->motorNode.WriteReq("MAIN.Axis_GoalPos", cmd)
        || !this->motorNode.WriteReq(
            "MAIN.startMove[" + to_string(physicalId + 1) + "]",
            true
        )){
        cerr << "Failed to move rail motor " << id << "." << endl;
        return false;
    }
    return true;
}

bool RailController::MoveSelectedMotorCmdAbsulote(double cmd){
    if(!this->useRail) return true;
    if(!this->isConnected) return false;
    if(!std::isfinite(cmd)){
        cerr << "Rail command must be finite." << endl;
        return false;
    }
    if(!this->motorNode.WriteReq("MAIN.Axis_GoalPos", cmd)
        || !this->motorNode.WriteReq(
            "MAIN.startMove[" + to_string(this->workingMotor + 1) + "]",
            true
        )){
        cerr << "Failed to move selected rail motor." << endl;
        return false;
    }
    return true;
}

bool RailController::CalibrationMotor(int index, double currentCmdPos){
    if(!this->useRail) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidRailIndex(index)){
        cerr << "Invalid rail motor index: " << index << endl;
        return false;
    }
    if(!std::isfinite(currentCmdPos)){
        cerr << "Rail calibration position must be finite." << endl;
        return false;
    }

    // Preserve the existing one-shot homing behavior. A busy/done handshake
    // requires PLC- and hardware-validated timeout semantics.
    if(!this->motorNode.WriteReq("MAIN.Axis_GoalPos", currentCmdPos)
        || !this->motorNode.WriteReq(
            "MAIN.bHomeSwitch[" + to_string(index + 1) + "]",
            true
        )){
        cerr << "Failed to calibrate rail motor " << index << "." << endl;
        return false;
    }
    return true;
}

bool RailController::GetMotorPosMeasured(vector<double>& result){
    result.clear();
    if(!this->useRail) return true;
    if(!this->isConnected) return false;

    result.reserve(static_cast<size_t>(this->railNumber));
    bool success = true;
    for(int i = 0; i < this->railNumber; i++){
        double actualPosition{0.0};
        if(!this->motorNode.ReadReq(
            "MAIN.actPos[" + to_string(i + 1) + "]",
            actualPosition
        )){
            cerr << "Failed to read rail motor " << i << " position." << endl;
            success = false;
        }
        cout << "Rail " << i << " position: " << actualPosition << endl;
        result.push_back(actualPosition);
    }
    return success;
}

vector<double> RailController::GetMotorPosMeasured(){
    vector<double> result;
    this->GetMotorPosMeasured(result);
    return result;
}

double RailController::GetMotorPosMeasuredById(int id){
    if(!this->useRail || !this->isConnected) return 0.0;
    if(!this->IsValidRailIndex(id)){
        cerr << "Invalid rail motor index: " << id << endl;
        return 0.0;
    }

    double actualPosition{0.0};
    if(!this->motorNode.ReadReq(
        "MAIN.actPos[" + to_string(id + 1) + "]",
        actualPosition
    )){
        cerr << "Failed to read rail motor " << id << " position." << endl;
        return 0.0;
    }
    return actualPosition;
}
