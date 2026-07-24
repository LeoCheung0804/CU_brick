#ifndef CableController_H
#define CableController_H

#include "TeknicNode.h"
#include "Logger.h"

using namespace std;
using namespace sFnd;
class CableController{
private:
    TeknicNode motorNode;
    int cableNumber{0};
    float absTrqLmt{0.0F};
    vector<INode*> nodeList;
    bool isOnline{false};
    bool useCable{false};
    bool isConnected{false};
    bool cleanupPending{false};
    int MILLIS_TO_NEXT_FRAME{50};
    Logger logger;
    bool IsValidCableIndex(int index) const;
public:
    explicit CableController(bool isOnline=false, bool useCable=false);
    bool Connect(int cableNumber);
    bool Disconnect();
    bool IsConnected() const;
    bool HasActiveResources() const;
    bool SetCableTrqByIndex(int index, float targetTrq, float tolerance);
    bool SetCableTrq(float targetTrq, float tolerance);
    bool MoveSingleMotorCmd(int index, int32_t cmd, bool absolute=true);
    bool MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute=true);
    bool MoveAllMotorCmd(int32_t cmd, bool absolute=true);
    bool CalibrationMotor(int index, int32_t currentCmdPos);
    double GetMotorPosMeasured(int index);
    double GetMotorTorqueMeasured(int index);
    bool StopAllMotor();
    void SetTrqLmt(float lmt);
    void ClearAlert();
};
#endif
