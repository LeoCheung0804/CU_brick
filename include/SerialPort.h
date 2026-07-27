#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>

class ISerialPort {
public:
    virtual ~ISerialPort() = default;

    virtual bool Connect(const std::string& portName) = 0;
    virtual bool Disconnect() = 0;
    virtual bool IsConnected() const = 0;
    virtual bool HasActiveResources() const = 0;
    virtual std::string Read() = 0;
    virtual bool Send(const std::string& command) = 0;
};

#endif
