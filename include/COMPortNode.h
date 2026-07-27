#ifndef COMPortNode_H
#define COMPortNode_H

#include "SerialPort.h"
#include <windows.h>
#include <string>

using namespace std;
class COMPortNode : public ISerialPort {
private:
    bool isOnline{false};
    bool isConnected{false};
    HANDLE hComm{INVALID_HANDLE_VALUE};
    int readTimeout{1000};
    
    // set communication port
    bool SetSerialParams();
public:
    explicit COMPortNode(bool isOnline=false);

    bool Connect(const string& portName) override;
    bool Disconnect() override;
    bool IsConnected() const override;
    bool HasActiveResources() const override;

    string Read() override;
    bool Send(const string& command) override;
};
#endif
