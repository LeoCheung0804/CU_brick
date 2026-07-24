#include "../include/GripperController.h"
#include <iostream>

GripperController::GripperController(bool isOnline, bool useGripper)
    : node(isOnline), isOnline(isOnline), useGripper(useGripper) {}

bool GripperController::Connect(const string& portName){
    if(this->HasActiveResources()){
        if(!this->Disconnect()){
            cout << "Error: Existing gripper connection could not be closed.\n";
            return false;
        }
    }
    this->isConnected = false;
    this->cleanupPending = false;
    if(!this->useGripper) return true;
    if(!this->node.Connect(portName)) { 
        cout << "Error: Failed to connect gripper. \n"; 
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    this->portName = portName;
    cout << "Gripper Controller Online." << endl;
    this->isConnected = true;
    this->cleanupPending = false;
    return true;
}

bool GripperController::Disconnect(){
    if(!this->HasActiveResources()) return true;
    if(!this->node.Disconnect()){
        cerr << "Failed to disconnect gripper; connection remains active." << endl;
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    cout << "Gripper Controller Offline." << endl;
    this->isConnected = false;
    this->cleanupPending = false;
    return true;
}

bool GripperController::IsConnected() const { return this->isConnected; }

bool GripperController::HasActiveResources() const {
    return this->isConnected || this->cleanupPending;
}

bool GripperController::Open(){
    if(!this->useGripper) return true;
    if(!this->isConnected) return false;
    this->sendStr = "(o,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    bool sent = node.Send(this->sendStr);
    sent = node.Send(this->sendStr) && sent;
    sent = node.Send(this->sendStr) && sent;
    if(!sent){
        cerr << "Failed to send gripper open command." << endl;
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    cout << "Gripper Opened. " << endl;
    return true;
}

bool GripperController::Close(){
    if(!this->useGripper) return true;
    if(!this->isConnected) return false;
    this->sendStr = "(c,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    bool sent = node.Send(this->sendStr);
    sent = node.Send(this->sendStr) && sent;
    sent = node.Send(this->sendStr) && sent;
    if(!sent){
        cerr << "Failed to send gripper close command." << endl;
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    cout << "Gripper Closed. " << endl;
    return true;
}
bool GripperController::Release(){
    if(!this->useGripper) return true;
    if(!this->isConnected) return false;
    this->sendStr = "(r,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    bool sent = node.Send(this->sendStr);
    sent = node.Send(this->sendStr) && sent;
    sent = node.Send(this->sendStr) && sent;
    if(!sent){
        cerr << "Failed to send gripper release command." << endl;
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    cout << "Gripper Released. " << endl;
    return true;
}

bool GripperController::Rotate(int angle){
    if(!this->useGripper) return true;
    if(!this->isConnected) return false;
    // assert(angle >= -180 && angle <= 180);
    angle = static_cast<int>(
        this->startAngle + (this->endAngle - this->startAngle) * angle / 180
    );
    this->sendStr = "(d,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    bool sent = node.Send(this->sendStr);
    sent = node.Send(this->sendStr) && sent;
    sent = node.Send(this->sendStr) && sent;
    if(!sent){
        cerr << "Failed to send gripper rotation command." << endl;
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    cout << "Gripper Rotated to: " << angle << endl;
    return true;
}
void GripperController::SetCalibrateStartAngle(int angle){
    if(!this->useGripper || !this->isConnected) return;
    // if(angle >= 200 || angle <= -200){
    //     cout << "Angle too large" << endl;
    //     return;
    // }
    this->sendStr = "(x,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    // node.Send(this->sendStr);
    this->startAngleTmp = static_cast<float>(angle);
    cout << "Gripper Rotated to: " << angle << endl;
}
void GripperController::SetCalibrateEndAngle(int angle){
    if(!this->useGripper || !this->isConnected) return;
    // if(angle >= 200 || angle <= -200){
        // cout << "Angle too large" << endl;
        // return;
    // }
    this->sendStr = "(y,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    // node.Send(this->sendStr);
    this->endAngleTmp = static_cast<float>(angle);
    cout << "Gripper Rotated to: " << angle << endl;
}

void GripperController::Calibrate(){
    if(!this->useGripper || !this->isConnected) return;
    // assert(angle >= -180 && angle <= 180);
    this->sendStr = "(w,";
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    // node.Send(this->sendStr);
    this->startAngle = this->startAngleTmp;
    this->endAngle = this->endAngleTmp;
    cout << "Gripper Calibrated " << endl;
}

void GripperController::Reset(){
    if(!this->useGripper || !this->isConnected) return;
    this->sendStr = "(h,  0 )";
    if(!node.Send(this->sendStr)){
        cerr << "Failed to send gripper reset command." << endl;
        return;
    }
    cout << "Sending Command: " << sendStr << " to gripper" << endl;
}

bool GripperController::Reconnect(){
    if(!this->useGripper) return true;
    if(this->HasActiveResources()){
        if(!this->Disconnect()){
            cout << "Error: Failed to close gripper before reconnecting.\n";
            return false;
        }
    }
    if(this->isOnline) Sleep(1000);
    cout << "Reconnecting Gripper Controller..." << endl;
    if(!this->node.Connect(this->portName)) {
        cout << "Error: Failed to reconnect gripper. \n"; 
        this->isConnected = false;
        this->cleanupPending = this->node.HasActiveResources();
        return false;
    }
    this->isConnected = true;
    this->cleanupPending = false;
    cout << "Gripper Controller Online." << endl;
    return true;
}
