
#include "../include/DeltaClient.h"

DeltaClient::DeltaClient(bool isOnline, bool useRail)
    : isOnline(isOnline), useRail(useRail) {
    // Set up server address
    this->serverAddr.sin_family = AF_INET;
    this->serverAddr.sin_port = htons(12345); // Use appropriate port number
    this->serverAddr.sin_addr.s_addr = inet_addr("192.168.0.1"); // Use RPi's IP address
}

DeltaClient::~DeltaClient(){
    this->Disconnect();
}

bool DeltaClient::EnsureSocket(){
    if(!this->isOnline) return true;
    if(!this->winsockInitialized){
        WSADATA wsaData;
        if(WSAStartup(MAKEWORD(2, 2), &wsaData) != 0){
            cout << "Failed to initialize Winsock" << endl;
            return false;
        }
        this->winsockInitialized = true;
    }
    if(this->udpSocket == INVALID_SOCKET){
        this->udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(this->udpSocket == INVALID_SOCKET){
            cout << "Failed to create socket" << endl;
            if(!this->ReleaseSocketResources()){
                cout << "Delta client socket cleanup remains pending" << endl;
            }
            return false;
        }
        const DWORD timeoutMs = 1000;
        if(setsockopt(
            this->udpSocket,
            SOL_SOCKET,
            SO_RCVTIMEO,
            reinterpret_cast<const char*>(&timeoutMs),
            sizeof(timeoutMs)
        ) == SOCKET_ERROR){
            cout << "Failed to set Delta client receive timeout" << endl;
            this->ReleaseSocketResources();
            return false;
        }
    }
    return true;
}

bool DeltaClient::ReleaseSocketResources(){
    if(this->udpSocket != INVALID_SOCKET){
        if(closesocket(this->udpSocket) == SOCKET_ERROR){
            cout << "Failed to close Delta client socket" << endl;
            return false;
        }
        this->udpSocket = INVALID_SOCKET;
    }
    if(this->winsockInitialized){
        if(WSACleanup() == SOCKET_ERROR){
            cout << "Failed to release Winsock" << endl;
            return false;
        }
        this->winsockInitialized = false;
    }
    return true;
}

bool DeltaClient::Connect(int motorPortNumber, int railNumber){
    if(!this->useRail) return true;
    if(this->isConnected || this->udpSocket != INVALID_SOCKET
        || this->winsockInitialized){
        if(!this->Disconnect())
            return false;
    }
    this->isConnected = false;
    if(railNumber <= 0 || railNumber > 4){
        cout << "Invalid rail count." << endl;
        return false;
    }
    this->railNumber = railNumber;
    if(!this->isOnline){
        this->isConnected = true;
        return true;
    }
    if(!this->EnsureSocket()) return false;
    
    const char* request = "connect";
    
    if (sendto(this->udpSocket, request, strlen(request), 0, (sockaddr*)&this->serverAddr, sizeof(this->serverAddr)) == SOCKET_ERROR) {
        cout << "Failed to send request to Delta Server" << endl;
        this->ReleaseSocketResources();
        system("pause");
        return false;
    }

    // Receive response
    char buffer[1024];
    int serverAddrLen = sizeof(this->serverAddr);
    int bytesReceived = recvfrom(this->udpSocket, buffer, sizeof(buffer), 0, (sockaddr*)&this->serverAddr, &serverAddrLen);
    
    if (bytesReceived == SOCKET_ERROR) {
        cout << "Failed to receive response from Delta Server" << endl;
        this->ReleaseSocketResources();
        system("pause");
        this->isConnected = false;
        return false;
    }

    cout << "Rail Controller Online." << endl;
    this->isConnected = true;
    return true;
}

bool DeltaClient::Disconnect(){
    const bool released = this->ReleaseSocketResources();
    this->isConnected = false;
    return released;
}

bool DeltaClient::IsConnected() const { return this->isConnected; }

void DeltaClient::SelectWorkingMotor(int index){
    if(!this->useRail || !this->isConnected) return;
    if(index < 0 || index >= this->railNumber){
        cout << "Invalid rail index: " << index << endl;
        return;
    }
    workingMotor = index;
}

bool DeltaClient::MoveSelectedMotorCmd(double cmd, bool absulote){
    if(!this->useRail) return true;
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;
    if(!absulote) return false;
    const char* request = "move";
    if (sendto(this->udpSocket, request, strlen(request), 0, (sockaddr*)&this->serverAddr, sizeof(this->serverAddr)) == SOCKET_ERROR) {
        cout << "Failed to send request to Delta Server" << endl;
        this->ReleaseSocketResources();
        this->isConnected = false;
        system("pause");
        return false;
    }

    // // Receive response
    // char buffer[1024];
    // int serverAddrLen = sizeof(this->serverAddr);
    // int bytesReceived = recvfrom(this->udpSocket, buffer, sizeof(buffer), 0, (sockaddr*)&this->serverAddr, &serverAddrLen);
    return true;
}

void DeltaClient::CalibrationMotor(int index, double currentCmdPos){
    cout << "Caution !!!!!! CalibrationMotor not implemented" << endl;
}

vector<int> DeltaClient::GetMotorPosMeasured(){
    cout << "Caution !!!!!! GetMotorPosMeasured not implemented" << endl;
    return vector<int>();
}
