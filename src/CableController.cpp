#include "../include/CableController.h"
#include <atomic>
#include <iostream>
#include <fstream>
#include <thread>
#include <windows.h>
#include <conio.h>
#include <queue>

CableController::CableController(bool isOnline, bool useCable)
    : motorNode(isOnline), isOnline(isOnline), useCable(useCable) {}

bool CableController::IsValidCableIndex(int index) const {
    return index >= 0 && index < this->cableNumber;
}

bool CableController::Connect(int cableNumber){
    if(this->HasActiveResources()){
        if(!this->Disconnect()){
            cerr << "Existing cable connection could not be safely closed." << endl;
            return false;
        }
    }
    this->isConnected = false;
    this->cleanupPending = false;
    if(!this->useCable) return true;
    if(cableNumber <= 0 || cableNumber > 8){
        cout << "Failed to connect cable motors: cable count must be between 1 and 8."
             << endl;
        return false;
    }

    this->cableNumber = cableNumber;
    if(this->isOnline && !this->logger.OpenFile("log\\trq.log")){
        cout << "Failed to open cable torque log." << endl;
        return false;
    }
    if(!this->motorNode.Connect(cableNumber)){
        cout << "Failed to connect cable motors. Exit programme." << endl;
        this->cleanupPending = this->motorNode.IsConnected();
        this->nodeList = this->motorNode.GetNodeList();
        if(!this->cleanupPending){
            this->nodeList.clear();
            this->logger.CloseFile();
        }
        return false;
    }
    this->nodeList = motorNode.GetNodeList();
    cout << "Cable Controller Online." << endl;
    this->isConnected = true;
    this->cleanupPending = false;
    return true;
}

bool CableController::Disconnect(){
    if(!this->HasActiveResources() && !this->motorNode.IsConnected()) return true;
    if(!this->motorNode.Disconnect()){
        cerr << "Failed to safely disconnect cable motors." << endl;
        this->isConnected = false;
        this->cleanupPending = true;
        return false;
    }
    this->logger.CloseFile();
    this->nodeList.clear();
    cout << "Cable Controller Offline." << endl;
    this->isConnected = false;
    this->cleanupPending = false;
    return true;
}

bool CableController::IsConnected() const { return this->isConnected; }

bool CableController::HasActiveResources() const {
    return this->isConnected || this->cleanupPending;
}

bool CableController::SetCableTrqByIndex(int index, float targetTrq, float tolerance){
    if(!this->useCable) return true;
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;
    if(!this->IsValidCableIndex(index)){
        cerr << "Invalid cable motor index: " << index << endl;
        return false;
    }
    cout << "Setting all cable torque to: " << targetTrq << endl;
        cout << "current torques: " << endl;
        motorNode.SetAccLmt(index, 200);
        // get current trq
        float currentTrq = motorNode.GetTrqMeasured(index);
        cout << "motor: " << index << ": " << currentTrq << "       " << endl;
        
        if(currentTrq > targetTrq){ 
            motorNode.SetVel(index, -10); 
            currentTrq = motorNode.GetTrqMeasured(index);
            while(currentTrq > targetTrq - tolerance){
                cout << "current torques: " << endl;
                // get current trq
                currentTrq = motorNode.GetTrqMeasured(index);
                cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                if(kbhit()){
                    motorNode.StopNode(index);
                    break;
                }
            }
        }
        const bool stopped = this->StopAllMotor();
        motorNode.SetAccLmt(index, 40000);
    cout << "Set cable " << index << " torque to: " << targetTrq << " finished." << endl;
    return stopped;
}

bool CableController::SetCableTrq(float targetTrq, float tolerance){
    if(!this->useCable) return true;
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;
    cout << "Setting all cable torque to: " << targetTrq << endl;
        vector<unsigned char> stopFlag(static_cast<size_t>(this->cableNumber), 0);
        bool moving = true;
        cout << "current torques: " << endl;
        float currentTrq = 0;
        for(int index = 0; index < cableNumber; index++){
            motorNode.SetAccLmt(index, 200);
            // get current trq
            currentTrq = motorNode.GetTrqMeasured(index);
            cout << "motor: " << index << ": " << currentTrq << "       " << endl;
            // check current trq and set move direction
            if(currentTrq > targetTrq - tolerance){ 
                motorNode.SetVel(index, -10); 
                stopFlag[index] = 0;
            }
        }
        // -5 < trq < -3.5
        string log = "";
        auto start = chrono::steady_clock::now();
        auto end = chrono::steady_clock::now();
        double dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
        
        while(moving){
            end = chrono::steady_clock::now();
            dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
            if(dif < 0){
                start = chrono::steady_clock::now();
                printf("\x1b[9A");
                moving = false;
                cout << "current torques: " << endl;
                log = "";
                
                for(int index = 0; index < cableNumber; index++){
                    // get current trq
                    currentTrq = motorNode.GetTrqMeasured(index);
                    if (currentTrq < targetTrq - tolerance || stopFlag[index]){
                        motorNode.StopNode(index); 
                        stopFlag[index] = 1;
                    }else{
                        moving = true;
                    }
                    cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                    log +=  to_string(currentTrq) + ",";
                }
                logger.LogInfo(log);
            }
            if(kbhit()){
                moving = false;
                break;
            }
        }
        const bool stopped = this->StopAllMotor();
        if(!stopped)
            cerr << "Torque adjustment ended without a verified cable stop."
                 << endl;
        for(int index = 0; index < this->cableNumber; index++){
            motorNode.SetAccLmt(index, 40000);
        }
        
    cout << "Set all cable torque to: " << targetTrq << " finished." << endl;
    return stopped;
}

void CableController::SetTrqLmt(float lmt){
    this->absTrqLmt = lmt;
}

void CableController::ClearAlert(){
    if(!this->useCable || !this->isConnected) return;
    motorNode.ClearAlert();
    cout << "Alert Cleared!" << endl;    
}

bool CableController::StopAllMotor(){
    if(!this->useCable) return true;
    if(!this->HasActiveResources()) return false;
    if(!motorNode.StopAll()){
        cerr << "Failed to stop all cable motors." << endl;
        this->isConnected = false;
        this->cleanupPending = true;
        return false;
    }
    cout << "All Node Stopped!" << endl;    
    return true;
}

bool CableController::MoveSingleMotorCmd(int index, int32_t cmd, bool absolute){
    // if(absolute){
    //     cout << "Move Single Cable Motor " << index << " Cmd to: " << cmd << endl;
    // }else{
    //     cout << "Move Single Cable Motor " << index << " Cmd by: " << cmd << endl;
    // }
    if(!this->useCable) return true;
    if(!this->isConnected){
        cerr << "Cable command rejected: controller is not connected." << endl;
        return false;
    }
    if(!this->IsValidCableIndex(index)){
        cerr << "Cable command rejected: invalid motor index " << index << "." << endl;
        return false;
    }
        ofstream myfile;
        try{
            motorNode.MoveToPosn(index, cmd, absolute);// absolute position?
            
            if (motorNode.NodeIsInAlert(index)) {
                myfile.open("log.txt", ios::app);
                myfile << "Alert from Motor [" << index << "]: "<< nodeList[index]->Status.Alerts.Value().bits <<"\n";
                myfile.close();
                return false;
            }
        }
        catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
            cout << "\nERROR: Motor [" << index << "] command failed.\n";  
            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            // quitType = 'e';
            tm *fn; time_t now = time(0); fn = localtime(&now);
            myfile.open("log.txt", ios::app);
            myfile << "\nERROR: Motor [" << index << "] command failed. " << fn->tm_hour << ":"<< fn->tm_min << ":" << fn->tm_sec <<"\n";
            myfile << "Caught error: addr="<< (int) theErr.TheAddr<<", err="<<hex<<theErr.ErrorCode <<"\nmsg="<<theErr.ErrorMsg<<"\n";
            myfile.close();
            return false;
        }
    return true;
}

bool CableController::MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute){
    if(!this->useCable) return true;
    if(!this->isConnected){
        cerr << "Cable commands rejected: controller is not connected." << endl;
        return false;
    }
    if(cmdList.size() != static_cast<size_t>(this->cableNumber)){
        cerr << "Cable commands rejected: expected " << this->cableNumber
             << " commands, received " << cmdList.size() << "." << endl;
        return false;
    }

    atomic_bool moveFailed{false};
    vector<thread> threadList;
    bool threadCreationFailed = false;
    int index = 0;
    // thread t1(&CableController::CheckTrq, this);
    try{
        threadList.reserve(cmdList.size());
        for(int32_t cmd : cmdList){
            threadList.emplace_back([this, index, cmd, absolute, &moveFailed](){
                try{
                    if(!this->MoveSingleMotorCmd(index, cmd, absolute))
                        moveFailed.store(true, memory_order_relaxed);
                }catch(const exception& e){
                    cerr << "Cable command worker failed: " << e.what() << endl;
                    moveFailed.store(true, memory_order_relaxed);
                }catch(...){
                    cerr << "Cable command worker failed." << endl;
                    moveFailed.store(true, memory_order_relaxed);
                }
            });
            index++;
        }
    }catch(const exception& e){
        cerr << "Failed to start all cable command workers: " << e.what()
             << endl;
        threadCreationFailed = true;
    }catch(...){
        cerr << "Failed to start all cable command workers." << endl;
        threadCreationFailed = true;
    }
    for(thread &thr : threadList){
        if(thr.joinable()) thr.join();
    }
    if(threadCreationFailed || moveFailed.load(memory_order_relaxed)){
        if(!this->StopAllMotor())
            cerr << "Cable command failure stop could not be verified." << endl;
        return false;
    }
    return true;
}

bool CableController::MoveAllMotorCmd(int32_t cmd, bool absolute){
    vector<int32_t> cmdList;
    for(int i = 0; i < this->cableNumber; i++){ cmdList.push_back(cmd); }
    return this->MoveAllMotorCmd(cmdList, absolute);
}

bool CableController::CalibrationMotor(int index, int32_t currentCmdPos){
    if(!this->useCable) return true;
    if(!this->isConnected || !this->IsValidCableIndex(index)){
        cerr << "Cable calibration rejected for motor index " << index << "." << endl;
        return false;
    }
    try{
        motorNode.SetCurrentPosn(index, currentCmdPos);
    }catch(sFnd::mnErr& theErr){
        printf(
            "Cable calibration failed: addr=%d, err=0x%08x, msg=%s\n",
            theErr.TheAddr,
            theErr.ErrorCode,
            theErr.ErrorMsg
        );
        return false;
    }
    return true;
}

double CableController::GetMotorPosMeasured(int index){
    if(!this->useCable) return 0;
    if(!this->isConnected || !this->IsValidCableIndex(index)){
        cerr << "Cable position read rejected for motor index " << index << "." << endl;
        return 0;
    }
        return (double) motorNode.GetCurrentPosn(index);
}

double CableController::GetMotorTorqueMeasured(int index){
    if(!this->useCable) return 0;
    if(!this->isConnected || !this->IsValidCableIndex(index)){
        cerr << "Cable torque read rejected for motor index " << index << "." << endl;
        return 0;
    }
        return (double) motorNode.GetTrqMeasured(index);
}
