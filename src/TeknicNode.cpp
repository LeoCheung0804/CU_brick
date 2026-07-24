#include "../include/TeknicNode.h"
#include <iostream>

using namespace std;

TeknicNode::TeknicNode(bool isOnline) : isOnline(isOnline) {}

bool TeknicNode::IsValidNodeIndex(int nodeIndex) const {
    return nodeIndex >= 0 && static_cast<size_t>(nodeIndex) < this->nodeList.size();
}

bool TeknicNode::CleanupConnection(){
    if(!this->isOnline){
        this->nodeList.clear();
        this->portsOpen = false;
        this->isConnected = false;
        return true;
    }

    bool nodesDisabled = true;
    for(INode* node : this->nodeList){
        if(node == nullptr) continue;
        try{
            node->Motion.NodeStop(STOP_TYPE_ABRUPT);
        }catch(mnErr& theErr){
            printf(
                "Error while stopping Teknic node: addr=%d, err=0x%08x, msg=%s\n",
                theErr.TheAddr,
                theErr.ErrorCode,
                theErr.ErrorMsg
            );
            nodesDisabled = false;
        }
        try{
            node->EnableReq(false);
        }catch(mnErr& theErr){
            printf(
                "Error while disabling Teknic node: addr=%d, err=0x%08x, msg=%s\n",
                theErr.TheAddr,
                theErr.ErrorCode,
                theErr.ErrorMsg
            );
            nodesDisabled = false;
        }
    }
    if(!nodesDisabled){
        this->isConnected = true;
        return false;
    }

    if(this->myMgr != nullptr && this->portsOpen){
        try{
            this->myMgr->PortsClose();
            this->portsOpen = false;
        }catch(mnErr& theErr){
            printf(
                "Error while closing Teknic ports: addr=%d, err=0x%08x, msg=%s\n",
                theErr.TheAddr,
                theErr.ErrorCode,
                theErr.ErrorMsg
            );
            this->isConnected = true;
            return false;
        }
    }
    this->nodeList.clear();
    this->isConnected = false;
    return true;
}

bool TeknicNode::Connect(int nodeNumber){
    if(this->isConnected || this->portsOpen){
        if(!this->Disconnect()){
            cerr << "Error: Existing Teknic connection could not be safely closed."
                 << endl;
            return false;
        }
    }
    this->isConnected = false;
    this->nodeNumber = nodeNumber;
    this->portCount = 0;
    this->nodeList.clear();
    this->comHubPorts.clear();

    if(nodeNumber <= 0){
        cout << "Error: Teknic node count must be positive." << endl;
        return false;
    }
    if(!this->isOnline){
        this->isConnected = true;
        return true;
    }

    try{
        cout << "Connecting to teknic nodes." << endl;
        this->myMgr = SysManager::Instance();
        SysManager::FindComHubPorts(comHubPorts);

        cout << "Found " <<comHubPorts.size() << " SC Hubs\n";
        for (portCount = 0; portCount < comHubPorts.size(); portCount++) {
            myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
        }
        if(portCount == 0) {
            cout << "Error: Unable to locate SC hub port\n";
            return false;
        }
        
        // A thrown PortsOpen call may have opened a subset of ports. Mark the
        // resource as possibly live before the call so rollback always tries
        // PortsClose.
        this->portsOpen = true;
        myMgr->PortsOpen(portCount);
        int connectedNodeNumber = 0;
        for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
            IPort &myPort = myMgr->Ports(i);
            connectedNodeNumber += myPort.NodeCount();
        }
        if(nodeNumber != connectedNodeNumber){
            cout << "Error: Connected motor number wrong!" << endl;
            this->CleanupConnection();
            return false;
        }
        for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
            IPort &myPort = myMgr->Ports(i);
            // myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL); // do we need this?
            // myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
            printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
        
            for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
                INode &theNode = myPort.Nodes(iNode);
                nodeList.push_back(&theNode);
                theNode.EnableReq(false); //Ensure Node is disabled before loading config file
                myMgr->Delay(200);

                printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
                printf("            userID: %s\n", theNode.Info.UserID.Value());
                printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
                printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
                printf("             Model: %s\n", theNode.Info.Model.Value());

                theNode.Status.AlertsClear();               //Clear Alerts on node 
                theNode.Motion.NodeStopClear();             //Clear Nodestops on Node               
                theNode.EnableReq(true);                    //Enable node 
                theNode.Motion.PosnMeasured.AutoRefresh(true);
                theNode.Motion.TrqMeasured.AutoRefresh(true);
                printf("Node %d enabled. ", iNode);

                theNode.AccUnit(INode::RPM_PER_SEC);        //Set the units for Acceleration to RPM/SEC
                theNode.VelUnit(INode::RPM);                //Set the units for Velocity to RPM
                theNode.Motion.AccLimit = 40000;           //100000 Set Acceleration Limit (RPM/Sec)
                theNode.Motion.NodeStopDecelLim = 5000;
                theNode.Motion.VelLimit = 3000;             //700 Set Velocity Limit (RPM)
                theNode.Info.Ex.Parameter(98, 1);           //enable interrupting move

                theNode.TrqUnit(theNode.PCT_MAX);
                theNode.Limits.TrqGlobal = 30;              // Set Trq Limit

                theNode.Motion.Adv.TriggerGroup(1);         //Set all nodes trigger group num. as 1
                cout << "AccLimit and VelLimit set." << endl;

                double timeout = myMgr->TimeStampMsec() + 2000; //TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
                //This will loop checking on the Real time values of the node's Ready status
                while (!theNode.Motion.IsReady()) {
                    if (myMgr->TimeStampMsec() > timeout) {
                        printf("Error: Timed out waiting for Node %d to enable\n", iNode);
                        this->CleanupConnection();
                        return false;
                    }
                }
            }
        }
        cout << "Teknic Nodes Connected (" << connectedNodeNumber << "/" << nodeNumber << ")" << endl;
        this->isConnected = true;
        return true;
    }
    catch(mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        printf("Error: Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        this->CleanupConnection();
        return false;
    }
}

bool TeknicNode::Disconnect(){
    return this->CleanupConnection();
}

bool TeknicNode::IsConnected() const {
    return this->isConnected;
}

vector<INode*> TeknicNode::GetNodeList(){
    return nodeList;
}

INode* TeknicNode::GetNode(int nodeIndex){
    if(this->IsValidNodeIndex(nodeIndex))
        return nodeList[nodeIndex];
    return nullptr;
}

void TeknicNode::SetAccLmt(int index, int val){ 
    if(!this->isOnline) return;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return;
    }
    this->nodeList[index]->Motion.AccLimit = val;
}

float TeknicNode::GetTrqMeasured(int index){
    if(!this->isOnline) return 0;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return 0;
    }
    return this->nodeList[index]->Motion.TrqMeasured.Value();
}

void TeknicNode::SetVel(int index, int val){
    if(!this->isOnline) return;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return;
    }
    this->nodeList[index]->Motion.MoveVelStart(val); 
}

void TeknicNode::StopNode(int index){
    if(!this->isOnline) return;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return;
    }
    this->nodeList[index]->Motion.NodeStop(STOP_TYPE_ABRUPT);
}

bool TeknicNode::StopAll(){
    if(!this->isOnline) return true;
    bool stopped = true;
    if(this->portsOpen){
        if(this->myMgr == nullptr || this->portCount <= 0)
            return false;
        // Port-level NodeStop reaches every attached node even when a
        // partially failed enumeration left nodeList incomplete.
        for(int portIndex = 0; portIndex < this->portCount; ++portIndex){
            try{
                IPort& port = this->myMgr->Ports(portIndex);
                const openStates state = port.OpenState();
                if(state == OPENED_ONLINE){
                    port.NodeStop(STOP_TYPE_ABRUPT);
                }else if(state == CLOSED
                    || state == PORT_UNAVAILABLE
                    || state == OPENED_SEARCHING){
                    // These states have no controllable nodes. Cleanup will
                    // still retry PortsClose for any partially opened set.
                    continue;
                }else{
                    cerr << "Teknic port " << portIndex
                         << " is in an indeterminate state: " << state << endl;
                    stopped = false;
                }
            }catch(mnErr& theErr){
                printf(
                    "Unable to stop Teknic port nodes: "
                    "addr=%d, err=0x%08x, msg=%s\n",
                    theErr.TheAddr,
                    theErr.ErrorCode,
                    theErr.ErrorMsg
                );
                stopped = false;
            }
        }
        return stopped;
    }

    // Fallback for any retained node handles after port state was cleared.
    for(INode* node : this->nodeList){
        if(node == nullptr) continue;
        try{
            node->Motion.NodeStop(STOP_TYPE_ABRUPT);
        }catch(mnErr& theErr){
            printf(
                "Unable to stop Teknic node: addr=%d, err=0x%08x, msg=%s\n",
                theErr.TheAddr,
                theErr.ErrorCode,
                theErr.ErrorMsg
            );
            stopped = false;
        }
    }
    return stopped;
}


void TeknicNode::ClearAlert(){
    if(!this->isOnline) return;
    for (INode* node : this->nodeList) {      
        node->EnableReq(true);                    //Enable node 
        node->Status.AlertsClear();               //Clear Alerts on node 
        node->Motion.NodeStopClear();             //Clear Nodestops on Node     
    }
}

void TeknicNode::MoveToPosn(int index, int32_t cmd, bool absolute){
    if(!this->isOnline) return;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return;
    }
    this->nodeList[index]->Motion.MovePosnStart(cmd, absolute, true);
}

bool TeknicNode::NodeIsInAlert(int index){
    if(!this->isOnline) return false;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return true;
    }
    return this->nodeList[index]->Status.Alerts.Value().isInAlert();
}

void TeknicNode::SetCurrentPosn(int index, int32_t cmd){
    if(!this->isOnline) return;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return;
    }
    this->nodeList[index]->Motion.PosnMeasured.Refresh();
    this->nodeList[index]->Motion.AddToPosition(-nodeList[index]->Motion.PosnMeasured.Value() + cmd);
}

int32_t TeknicNode::GetCurrentPosn(int index){
    if(!this->isOnline) return 0;
    if(!this->IsValidNodeIndex(index)){
        cerr << "Invalid Teknic node index: " << index << endl;
        return 0;
    }
    return this->nodeList[index]->Motion.PosnMeasured.Value();
}
