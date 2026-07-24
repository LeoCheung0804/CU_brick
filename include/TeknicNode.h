#ifndef TeknicNode_H
#define TeknicNode_H
#include "../Dependencies/sFoundation20/inc/pubSysCls.h"
#include <vector>
#include <string>

using namespace std;
using namespace sFnd;
class TeknicNode{
private:
    bool isOnline{false};
    bool isConnected{false};
    bool portsOpen{false};
    int nodeNumber{0};
    int portCount{0};
    vector<string> comHubPorts;
    vector<INode*> nodeList; // create a list for each node
    SysManager* myMgr{nullptr};
    bool IsValidNodeIndex(int nodeIndex) const;
    bool CleanupConnection();
public:
    explicit TeknicNode(bool isOnline=false);
    /// @brief Scan the network, find all connected sc hub and all connected teknic motors.
    /// @return bool. Whether the connection has been established.
    bool Connect(int nodeNumber);
    bool Disconnect();
    bool IsConnected() const;
    
    INode* GetNode(int nodeIndex);
    vector<INode*> GetNodeList();
    void SetAccLmt(int index, int val);
    float GetTrqMeasured(int index);
    void SetVel(int index, int val);
    void StopNode(int index);
    bool StopAll();
    void ClearAlert();
    void MoveToPosn(int index, int32_t cmd, bool absolute);
    bool NodeIsInAlert(int index);
    void SetCurrentPosn(int index, int32_t cmd);
    int32_t GetCurrentPosn(int index);
};

#endif
