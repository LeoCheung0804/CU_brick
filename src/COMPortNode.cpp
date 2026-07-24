#include "../include/COMPortNode.h"
#include <algorithm>
#include <iostream>
#include <thread>

COMPortNode::COMPortNode(bool isOnline) : isOnline(isOnline) {}

bool COMPortNode::SetSerialParams(){
    if(!this->isOnline) return true;
    // Set parameters for serial port
    DCB dcbSerialParams = { 0 }; // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    BOOL status = GetCommState(this->hComm, &dcbSerialParams); // retreives the current settings
    if (status == FALSE){ cout << "Error in GetCommState()\n"; return false; }

    dcbSerialParams.BaudRate = CBR_57600;// Setting BaudRate
    dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
    dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
    dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None

    status = SetCommState(this->hComm, &dcbSerialParams);
    if (status == FALSE){ cout << "Error! in Setting DCB Structure\n"; return false; }
    else{
        printf("   Setting DCB Structure Successful\n");
        printf("       Baudrate = %d\n", dcbSerialParams.BaudRate);
        printf("       ByteSize = %d\n", dcbSerialParams.ByteSize);
        printf("       StopBits = %d\n", dcbSerialParams.StopBits);
        printf("       Parity   = %d\n", dcbSerialParams.Parity);
        cout << endl;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(this->hComm, &timeouts) == FALSE){ cout << "Error! in Setting Time Outs\n"; return false; }
    // Set recieve mask                
    if (!(bool)SetCommMask(this->hComm, EV_RXCHAR)){
        cout << "Error! in Setting CommMask\n";
        return false;
    }
    
    return true;
}

bool COMPortNode::Connect(const string& portName){
    if(this->isConnected || this->hComm != INVALID_HANDLE_VALUE){
        if(!this->Disconnect()){
            cout << "Error: Existing serial connection could not be closed.\n";
            return false;
        }
    }
    this->isConnected = false;
    if(!this->isOnline) {
        this->isConnected = true;
        return true;
    }

    if(this->hComm != INVALID_HANDLE_VALUE){
        CloseHandle(this->hComm);
        this->hComm = INVALID_HANDLE_VALUE;
    }

    cout << "Connecting to BLE Device on port: " << portName << endl;;
    const char* cArray = portName.c_str();
    this->hComm = CreateFile(cArray, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (this->hComm == INVALID_HANDLE_VALUE){ 
        cout << "Error: " << portName << " cannot be opened.\n"; 
        return false;
    }
    cout << portName << " opened.\n";

    if (!SetSerialParams()) { 
        if(CloseHandle(this->hComm)){
            this->hComm = INVALID_HANDLE_VALUE;
        }else{
            cout << "Error: Serial setup failed and the port could not be closed: "
                 << GetLastError() << "\n";
        }
        return false; 
    }
    Sleep(2000);
    cout << "Connected to BLE Device on port: " << portName << endl;
    this->isConnected = true;
    return true;
}

bool COMPortNode::Disconnect(){
    if(!this->isOnline){
        this->hComm = INVALID_HANDLE_VALUE;
        this->isConnected = false;
        return true;
    }

    if(this->hComm == INVALID_HANDLE_VALUE){
        this->isConnected = false;
        return true;
    }

    if(this->isConnected && !this->Send("(z,    )")){
        cout << "Error: Serial shutdown command failed; port remains open.\n";
        return false;
    }
    if(!CloseHandle(this->hComm)){
        cout << "Error: Serial port could not be closed: " << GetLastError()
             << "\n";
        return false;
    }
    this->hComm = INVALID_HANDLE_VALUE;
    this->isConnected = false;
    return true;
}

bool COMPortNode::IsConnected() const {
    return this->isConnected;
}

bool COMPortNode::HasActiveResources() const {
    return this->isConnected
        || (this->isOnline && this->hComm != INVALID_HANDLE_VALUE);
}

bool COMPortNode::Send(const string& command){
    if(!this->isConnected){
        cout << "Serial command rejected: port is not connected." << endl;
        return false;
    }
    if(command.size() != 8){
        cout << "Serial command rejected: expected exactly 8 bytes, received "
             << command.size() << "." << endl;
        return false;
    }
    if(!this->isOnline) return true;

    char buffer[8] = {};
    copy(command.begin(), command.end(), buffer);
    DWORD dNoOfBytesWritten = 0;
    if (!(bool)WriteFile(this->hComm, buffer, 8, &dNoOfBytesWritten, NULL)){ 
        cout << "Arduino writing error: " << GetLastError() << endl; 
        return false;
    }    
    if(dNoOfBytesWritten != 8){
        cout << "Arduino writing error: wrote " << dNoOfBytesWritten
             << " of 8 bytes." << endl;
        return false;
    }
    cout << "Sent: " << command << endl;
    return true;
    // return Read();
    // Read(); // need this?
}

string COMPortNode::Read(){
    if(!this->isOnline || !this->isConnected) return "";
    DWORD BytesRead{0};
    char tmp{0};
    string msg = "";

    const auto deadline = chrono::steady_clock::now()
        + chrono::milliseconds(this->readTimeout);
    while(chrono::steady_clock::now() < deadline){
        if(!ReadFile(this->hComm, &tmp, sizeof(tmp), &BytesRead, NULL)){
            cout << "Serial read failed: " << GetLastError() << "\n";
            return "";
        }
        if(BytesRead > 0){
            msg.push_back(tmp);
            break;
        }
    }

    constexpr size_t maxMessageLength = 1024;
    do{
        if(!ReadFile(this->hComm, &tmp, sizeof(tmp), &BytesRead, NULL)){
            cout << "Serial read failed: " << GetLastError() << "\n";
            return msg;
        }
        if(BytesRead > 0)
            msg.push_back(tmp);
    }while(BytesRead > 0
        && chrono::steady_clock::now() < deadline
        && msg.size() < maxMessageLength);
    return msg;

}
