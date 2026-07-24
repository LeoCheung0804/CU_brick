#include "../include/BrakeController.h"
#include <iostream>
#define RAIL_BRAKE_INDEX_START 0
#define CABLE_BRAKE_INDEX_START 4
#define RAIL_BRAKE_ALL 8
#define CABLE_BRAKE_ALL 9
BrakeController::BrakeController(bool isOnline)
    : node(isOnline), isOnline(isOnline) {}

bool BrakeController::IsValidRailBrakeIndex(int index) const {
    return index >= 0 && index < this->railBrakeNum;
}

bool BrakeController::IsValidCableBrakeIndex(int index) const {
    return index >= 0 && index < this->cableBrakeNum;
}

bool BrakeController::Connect(const string& portName){
    if(this->HasActiveResources()){
        if(!this->Disconnect()){
            cout << "Error: Existing brake connection could not be closed.\n";
            return false;
        }
    }
    this->isConnected = false;
    this->cleanupPending = false;
    if(!this->useRailBrake && !this->useCableBrake) return true;
    if(!this->node.Connect(portName)) { 
        cout << "Error: Failed to connect Brake. \n"; 
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    if(this->isOnline) Sleep(2000);
    cout << "Brake Controller Online." << endl;
    this->isConnected = true;
    this->cleanupPending = false;
    return true;
}

bool BrakeController::Disconnect(){
    if(!this->HasActiveResources()) return true;
    if(!this->node.Disconnect()){
        cerr << "Failed to disconnect brake controller; connection remains active."
             << endl;
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    cout << "Brake Controller Offline." << endl;
    this->isConnected = false;
    this->cleanupPending = false;
    return true;
}

bool BrakeController::UseCableBrake(int num){
    if(num < 0 || num > 4){
        cerr << "Cable brake count must be between 0 and 4." << endl;
        this->cableBrakeNum = 0;
        this->useCableBrake = false;
        return false;
    }
    this->cableBrakeNum = num;
    this->useCableBrake = this->cableBrakeNum > 0;
    return true;
}

bool BrakeController::UseRailBrake(int num){
    if(num < 0 || num > 4){
        cerr << "Rail brake count must be between 0 and 4." << endl;
        this->railBrakeNum = 0;
        this->useRailBrake = false;
        return false;
    }
    this->railBrakeNum = num;
    this->useRailBrake = this->railBrakeNum > 0;
    return true;
}

bool BrakeController::IsConnected() const { return this->isConnected; }

bool BrakeController::HasActiveResources() const {
    return this->isConnected || this->cleanupPending;
}

bool BrakeController::OpenRailBrakeByIndex(int index){
    if(!this->useRailBrake) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidRailBrakeIndex(index)){
        cerr << "Invalid rail brake index: " << index << endl;
        return false;
    }
    this->sendStr[3] = '0';
    this->sendStr[1] = '0' + RAIL_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    if(!this->node.Send(this->sendStr)){
        cerr << "Failed to open rail brake " << index << "." << endl;
        return false;
    }
    if(this->isOnline) Sleep(100);
    cout << "Rail Brake " << index << " Opened." << endl;
    return true;
}

bool BrakeController::CloseRailBrakeByIndex(int index){
    if(!this->useRailBrake) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidRailBrakeIndex(index)){
        cerr << "Invalid rail brake index: " << index << endl;
        return false;
    }
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + RAIL_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    if(!this->node.Send(this->sendStr)){
        cerr << "Failed to close rail brake " << index << "." << endl;
        return false;
    }
    if(this->isOnline) Sleep(100);
    cout << "Rail Brake " << index << " Closed." << endl;
    return true;
}

bool BrakeController::OpenAllRailBrake(){
    if(!this->useRailBrake) return true;
    if(!this->isConnected) return false;
    bool success = true;
    for(int i = 0; i < this->railBrakeNum; i++){
        if(!this->OpenRailBrakeByIndex(i)){
            success = false;
        }
    }
    if(this->isOnline) Sleep(1000);
    if(success){
        cout << "ALL Rail Brake Opened." << endl;
    }else{
        cerr << "Failed to open one or more rail brakes." << endl;
        if(!this->CloseAllRailBrake()){
            cerr << "CRITICAL: Failed to re-close rail brakes after a partial open."
                 << endl;
        }
    }
    return success;
}

bool BrakeController::CloseAllRailBrake(){
    if(!this->useRailBrake) return true;
    if(!this->isConnected) return false;
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + RAIL_BRAKE_ALL;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    if(!this->node.Send(this->sendStr)){
        cerr << "Failed to close all rail brakes." << endl;
        return false;
    }
    if(this->isOnline) Sleep(100);
    cout << "ALL Rail Brake Closed." << endl;
    return true;
}

bool BrakeController::OpenCableBrakeByIndex(int index){
    if(!this->useCableBrake) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidCableBrakeIndex(index)){
        cerr << "Invalid cable brake index: " << index << endl;
        return false;
    }
    this->sendStr[3] = '0';
    this->sendStr[1] = '0' + CABLE_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to cable brakes" << endl;
    if(!this->node.Send(this->sendStr)){
        cerr << "Failed to open cable brake " << index << "." << endl;
        return false;
    }
    if(this->isOnline) Sleep(100);
    cout << "Cable Brake " << index << " Opened." << endl;
    return true;
}

bool BrakeController::CloseCableBrakeByIndex(int index){
    if(!this->useCableBrake) return true;
    if(!this->isConnected) return false;
    if(!this->IsValidCableBrakeIndex(index)){
        cerr << "Invalid cable brake index: " << index << endl;
        return false;
    }
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + CABLE_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to cable brakes" << endl;
    if(!this->node.Send(this->sendStr)){
        cerr << "Failed to close cable brake " << index << "." << endl;
        return false;
    }
    if(this->isOnline) Sleep(100);
    cout << "Cable Brake " << index << " Closed." << endl;
    return true;
}

bool BrakeController::OpenAllCableBrake(){
    if(!this->useCableBrake) return true;
    if(!this->isConnected) return false;
    bool success = true;
    for(int i = 0; i < this->cableBrakeNum; i++){
        if(!this->OpenCableBrakeByIndex(i)){
            success = false;
        }
    }
    if(this->isOnline) Sleep(1000);
    if(success){
        cout << "ALL Cable Brake Opened." << endl;
    }else{
        cerr << "Failed to open one or more cable brakes." << endl;
        if(!this->CloseAllCableBrake()){
            cerr << "CRITICAL: Failed to re-close cable brakes after a partial open."
                 << endl;
        }
    }
    return success;
}

bool BrakeController::CloseAllCableBrake(){
    if(!this->useCableBrake) return true;
    if(!this->isConnected) return false;
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + CABLE_BRAKE_ALL;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    if(!this->node.Send(this->sendStr)){
        cerr << "Failed to close all cable brakes." << endl;
        return false;
    }
    if(this->isOnline) Sleep(100);
    cout << "ALL Cable Brake Closed." << endl;
    return true;
}
