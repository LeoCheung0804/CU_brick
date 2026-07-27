#ifndef BrakeController_H
#define BrakeController_H

#include "SerialPort.h"
#include <memory>
#include <string>

using namespace std;
class BrakeController{
private:
    unique_ptr<ISerialPort> node;
    bool isOnline{false};
    bool useRailBrake{false};
    bool useCableBrake{false};
    
    string sendStr{"(0:0)   "};
    bool isConnected{false};
    bool cleanupPending{false};

    int railBrakeNum{0};
    int cableBrakeNum{0};
    bool IsValidRailBrakeIndex(int index) const;
    bool IsValidCableBrakeIndex(int index) const;
    
public:
    explicit BrakeController(bool isOnline=false);
    // Injection seam for deterministic tests and alternate serial transports.
    BrakeController(bool isOnline, unique_ptr<ISerialPort> serialPort);

    bool Connect(const string& portName);
    
    bool Disconnect();
    bool IsConnected() const;
    bool HasActiveResources() const;
    
    bool UseRailBrake(int val);
    bool UseCableBrake(int val);

    bool OpenRailBrakeByIndex(int index);
    bool CloseRailBrakeByIndex(int index);

    bool OpenAllRailBrake();
    bool CloseAllRailBrake();
        
    bool OpenCableBrakeByIndex(int index);
    bool CloseCableBrakeByIndex(int index);

    bool OpenAllCableBrake();
    bool CloseAllCableBrake();

};
#endif
