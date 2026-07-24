#ifndef DeltaClient_H
#define DeltaClient_H

#include "vector"
#include "../include/Logger.h"
#include "../include/Robot.h"
#include "../include/TrajectoryGenerator.h"
#include <iostream>
#include <string>
#include <fstream>
#include "../tools/json.hpp"
#include <assert.h>
#pragma comment(lib, "User32.lib")

#pragma comment(lib, "ws2_32.lib")


using json = nlohmann::json;
using namespace std;
class DeltaClient{
private:
    SOCKET udpSocket{INVALID_SOCKET};
    sockaddr_in serverAddr{};
    int railNumber{0};
    bool isOnline{false};
    bool winsockInitialized{false};
    int MILLIS_TO_NEXT_FRAME{50};
    string sendStr;
    int workingMotor{0};
    bool isConnected{false};
    bool useRail{false};
    bool EnsureSocket();
    bool ReleaseSocketResources();
public:
    explicit DeltaClient(bool isOnline=false, bool useRail=false);
    ~DeltaClient();
    DeltaClient(const DeltaClient&) = delete;
    DeltaClient& operator=(const DeltaClient&) = delete;

    bool Connect(int motorPortNumber, int railNumber);
    bool Disconnect();
    bool IsConnected() const;
    void CalibrationMotor(int index, double currentCmdPos);
    void SelectWorkingMotor(int index);
    bool MoveSelectedMotorCmd(double cmd, bool absolute=true);
    vector<int> GetMotorPosMeasured();
};
#endif
