#ifndef RailController_H
#define RailController_H

#include "TwincatADSNode.h"
#include "vector"

using namespace std;
class RailController{
private:
    TwincatADSNode motorNode;
    int railNumber{0};
    bool isOnline{false};
    int MILLIS_TO_NEXT_FRAME{50};
    int workingMotor{0};
    bool isConnected{false};
    bool cleanupPending{false};
    bool useRail{false};
    bool IsValidRailIndex(int index) const;
    int ToPhysicalRailIndex(int logicalIndex) const;
public:
    explicit RailController(bool isOnline=false, bool useRail=false);
    bool Connect(int motorPortNumber, int railNumber);
    bool Disconnect();
    bool IsConnected() const;
    bool HasActiveResources() const;
    bool CalibrationMotor(int index, double currentCmdPos);
    bool SelectWorkingMotor(int index);
    bool MoveSelectedMotorCmdAbsulote(double cmd);
    bool MoveMotorCmdAbsuloteById(int id, double cmd);
    bool GetMotorPosMeasured(vector<double>& result);
    vector<double> GetMotorPosMeasured();
    double GetMotorPosMeasuredById(int id);
};
#endif
