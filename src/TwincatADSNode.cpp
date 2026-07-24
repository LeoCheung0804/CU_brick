#include "../include/TwincatADSNode.h"
#include <iostream>

TwincatADSNode::TwincatADSNode(bool isOnline) : isOnline(isOnline) {}

bool TwincatADSNode::TryGetHandle(const string& name, unsigned long& handle) const {
    const auto iterator = this->handlers.find(name);
    if(iterator == this->handlers.end()){
        cerr << "Error: TwinCAT ADS handle not found for " << name << '\n';
        return false;
    }
    handle = iterator->second;
    return true;
}

bool TwincatADSNode::Connect(int port){
    if(this->isConnected || this->pAddr != nullptr){
        if(!this->Disconnect()){
            cerr << "Error: Existing TwinCAT connection could not be safely closed."
                 << endl;
            return false;
        }
    }
    this->isConnected = false;
    this->powerEnabled = false;
    this->handlers.clear();
    this->pAddr = nullptr;

    if(port <= 0 || port > 65535){
        cout << "Error: Invalid TwinCAT ADS port: " << port << '\n';
        return false;
    }
    if(!this->isOnline){
        this->isConnected = true;
        return true;
    }

    cout << "Connecting to Twincat ADS Nodes." << endl;
    cout << "Opening Ads Port" << endl;
    const long clientPort = AdsPortOpen();
    if(clientPort == 0){
        cout << "Error: AdsPortOpen failed.\n";
        return false;
    }

    this->pAddr = &this->Addr;
    cout << "Getting Local Address" << endl;
    long nErr = AdsGetLocalAddress(this->pAddr);
    if(nErr){
        cout << "Error: AdsGetLocalAddress: " << nErr << '\n';
        if(!this->Disconnect()){
            cerr << "CRITICAL: TwinCAT address lookup failed and the ADS port "
                 << "could not be closed." << endl;
        }
        return false;
    }
    this->pAddr->port = static_cast<unsigned short>(port);
    cout << "Address Get" << endl;

    // Resolve every symbol before enabling motor power. This prevents a
    // partially initialized connection from leaving the PLC powered.
    for(size_t i = 0; i < sizeof(this->adsVarNames) / sizeof(this->adsVarNames[0]); ++i){
        unsigned long symbolHandle{0};
        string& symbolName = this->adsVarNames[i];
        nErr = AdsSyncReadWriteReq(
            this->pAddr,
            ADSIGRP_SYM_HNDBYNAME,
            0x0,
            sizeof(symbolHandle),
            &symbolHandle,
            static_cast<unsigned long>(symbolName.size() + 1),
            &symbolName[0]
        );
        if(nErr){
            cout << "Error: AdsSyncReadWriteReq: " << nErr
                 << ", in creating handler for " << symbolName << "\n";
            if(!this->Disconnect()){
                cerr << "CRITICAL: TwinCAT symbol initialization failed and "
                     << "ADS cleanup is incomplete." << endl;
            }
            return false;
        }
        this->handlers[symbolName] = symbolHandle;
    }

    char powerSymbol[] = "MAIN.power";
    unsigned long powerHandle{0};
    nErr = AdsSyncReadWriteReq(
        this->pAddr,
        ADSIGRP_SYM_HNDBYNAME,
        0x0,
        sizeof(powerHandle),
        &powerHandle,
        sizeof(powerSymbol),
        powerSymbol
    );
    if(nErr){
        cout << "Error: AdsSyncReadWriteReq: " << nErr
             << ", in creating handler for MAIN.power\n";
        this->Disconnect();
        return false;
    }

    bool enablePower = true;
    // A failed remote write can be indeterminate. Treat power as possibly
    // enabled until an explicit disable succeeds.
    this->powerEnabled = true;
    nErr = AdsSyncWriteReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        powerHandle,
        sizeof(enablePower),
        &enablePower
    );
    if(nErr){
        cout << "Error: AdsSyncWriteReq: " << nErr << '\n';
        AdsSyncWriteReq(
            this->pAddr,
            ADSIGRP_SYM_RELEASEHND,
            0,
            sizeof(powerHandle),
            &powerHandle
        );
        this->Disconnect();
        return false;
    }
    cout << "Linear rail motors enabled.\n";

    nErr = AdsSyncWriteReq(
        this->pAddr,
        ADSIGRP_SYM_RELEASEHND,
        0,
        sizeof(powerHandle),
        &powerHandle
    );
    if(nErr){
        cout << "Error: Failed to release MAIN.power handle: " << nErr << '\n';
        this->Disconnect();
        return false;
    }

    this->isConnected = true;
    cout << "Completed creating handler list of " << handlers.size() << endl;
    return true;
}

bool TwincatADSNode::Disconnect(){
    if(!this->isOnline){
        this->handlers.clear();
        this->pAddr = nullptr;
        this->powerEnabled = false;
        this->isConnected = false;
        return true;
    }
    if(this->pAddr == nullptr){
        this->handlers.clear();
        this->powerEnabled = false;
        this->isConnected = false;
        return true;
    }

    bool cleanupSucceeded = true;
    if(this->powerEnabled){
        char powerSymbol[] = "MAIN.power";
        unsigned long powerHandle{0};
        long nErr = AdsSyncReadWriteReq(
            this->pAddr,
            ADSIGRP_SYM_HNDBYNAME,
            0x0,
            sizeof(powerHandle),
            &powerHandle,
            sizeof(powerSymbol),
            powerSymbol
        );
        if(nErr){
            cerr << "CRITICAL: Unable to resolve MAIN.power while disconnecting: "
                 << nErr << ". ADS connection remains open." << endl;
            this->isConnected = true;
            return false;
        }

        bool enablePower = false;
        nErr = AdsSyncWriteReq(
            this->pAddr,
            ADSIGRP_SYM_VALBYHND,
            powerHandle,
            sizeof(enablePower),
            &enablePower
        );
        if(nErr){
            cerr << "CRITICAL: Unable to disable linear rail motors: " << nErr
                 << ". ADS connection remains open." << endl;
            AdsSyncWriteReq(
                this->pAddr,
                ADSIGRP_SYM_RELEASEHND,
                0,
                sizeof(powerHandle),
                &powerHandle
            );
            this->isConnected = true;
            return false;
        }
        this->powerEnabled = false;
        cout << "Linear rail motors disabled.\n";

        const long releaseError = AdsSyncWriteReq(
            this->pAddr,
            ADSIGRP_SYM_RELEASEHND,
            0,
            sizeof(powerHandle),
            &powerHandle
        );
        if(releaseError){
            cout << "Error: Failed to release MAIN.power handle: "
                 << releaseError << '\n';
            cleanupSucceeded = false;
        }
    }

    for(const auto& entry : this->handlers){
        unsigned long handle = entry.second;
        const long releaseError = AdsSyncWriteReq(
            this->pAddr,
            ADSIGRP_SYM_RELEASEHND,
            0,
            sizeof(handle),
            &handle
        );
        if(releaseError){
            cout << "Error: Failed to release ADS handle for " << entry.first
                 << ": " << releaseError << '\n';
            cleanupSucceeded = false;
        }
    }
    this->handlers.clear();

    const long closeError = AdsPortClose();
    if(closeError){
        cout << "Error: AdsPortClose: " << closeError << '\n';
        this->isConnected = true;
        return false;
    }
    this->pAddr = nullptr;
    this->isConnected = false;
    return cleanupSucceeded;
}

bool TwincatADSNode::IsConnected() const {
    return this->isConnected;
}

bool TwincatADSNode::WriteReq(const string& handleName, double data){
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;

    unsigned long handle{0};
    if(!this->TryGetHandle(handleName, handle)) return false;
    const long nErr = AdsSyncWriteReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        handle,
        sizeof(data),
        &data
    );
    if(nErr){
        cout << "Error: AdsSyncWriteReq: " << nErr << '\n';
        return false;
    }
    return true;
}

bool TwincatADSNode::WriteReq(const string& handleName, bool data){
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;

    unsigned long handle{0};
    if(!this->TryGetHandle(handleName, handle)) return false;
    const long nErr = AdsSyncWriteReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        handle,
        sizeof(data),
        &data
    );
    if(nErr){
        cout << "Error: AdsSyncWriteReq: " << nErr << '\n';
        return false;
    }
    return true;
}

bool TwincatADSNode::ReadReq(const string& handleName, bool& data){
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;

    unsigned long handle{0};
    if(!this->TryGetHandle(handleName, handle)) return false;
    const long nErr = AdsSyncReadReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        handle,
        sizeof(data),
        &data
    );
    if(nErr){
        cout << "Error: AdsSyncReadReq: " << nErr << '\n';
        return false;
    }
    return true;
}

bool TwincatADSNode::ReadReq(const string& handleName, double& data){
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;

    unsigned long handle{0};
    if(!this->TryGetHandle(handleName, handle)) return false;
    const long nErr = AdsSyncReadReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        handle,
        sizeof(data),
        &data
    );
    if(nErr){
        cout << "Error: AdsSyncReadReq: " << nErr << '\n';
        return false;
    }
    return true;
}

bool TwincatADSNode::ReadReq(const string& handleName, int& data){
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;

    unsigned long handle{0};
    if(!this->TryGetHandle(handleName, handle)) return false;
    const long nErr = AdsSyncReadReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        handle,
        sizeof(data),
        &data
    );
    if(nErr){
        cout << "Error: AdsSyncReadReq: " << nErr << '\n';
        return false;
    }
    return true;
}

bool TwincatADSNode::ReadReq(const string& handleName, float& data){
    if(!this->isConnected) return false;
    if(!this->isOnline) return true;

    unsigned long handle{0};
    if(!this->TryGetHandle(handleName, handle)) return false;
    const long nErr = AdsSyncReadReq(
        this->pAddr,
        ADSIGRP_SYM_VALBYHND,
        handle,
        sizeof(data),
        &data
    );
    if(nErr){
        cout << "Error: AdsSyncReadReq: " << nErr << '\n';
        return false;
    }
    return true;
}
