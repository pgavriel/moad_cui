#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

// setup class SimpleSerial similar to the windows version
class SimpleSerial {
    private:
        int fd_; // file descriptor for serial port

    public:
        SimpleSerial(const char* portname, int baudrate);
        std::string ReadSerialPort(int reply_wait_time, const std::string& syntax_type);
        bool WriteSerialPort(const char *data_sent);
        bool CloseSerialPort();
        ~SimpleSerial();
        bool connected_;
};
