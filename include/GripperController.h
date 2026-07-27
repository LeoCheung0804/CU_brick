#ifndef GripperController_H
#define GripperController_H

#include "SerialPort.h"
#include <memory>
#include <string>
#include <map>

using namespace std;
class GripperController{
private:
    unique_ptr<ISerialPort> node;
    bool isOnline{false};
    bool useGripper{false};
    int MILLIS_TO_NEXT_FRAME{50};
    string sendStr;
    bool isConnected{false};
    bool cleanupPending{false};
    string portName;
    float startAngle{-137.0F};
    float endAngle{43.0F};
    float startAngleTmp{-137.0F};
    float endAngleTmp{43.0F};
public:
    explicit GripperController(bool isOnline=false, bool useGripper=false);
    // Injection seam for deterministic tests and alternate serial transports.
    GripperController(
        bool isOnline,
        bool useGripper,
        unique_ptr<ISerialPort> serialPort
    );

    bool Connect(const string& portName);
    bool Disconnect();
    bool IsConnected() const;
    bool HasActiveResources() const;

    bool Open();
    bool Close();
    bool Rotate(int angle);
    bool Release();
    void SetCalibrateStartAngle(int angle);
    void SetCalibrateEndAngle(int angle);
    void Calibrate();
    void Reset();
    bool Reconnect();
};
#endif
