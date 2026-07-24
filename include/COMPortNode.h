#ifndef COMPortNode_H
#define COMPortNode_H

#include <windows.h>
#include <string>

using namespace std;
class COMPortNode{
private:
    bool isOnline{false};
    bool isConnected{false};
    HANDLE hComm{INVALID_HANDLE_VALUE};
    int readTimeout{1000};
    
    // set communication port
    bool SetSerialParams();
public:
    explicit COMPortNode(bool isOnline=false);

    bool Connect(const string& portName);
    bool Disconnect();
    bool IsConnected() const;
    bool HasActiveResources() const;

    string Read();
    bool Send(const string& command);
};
#endif
