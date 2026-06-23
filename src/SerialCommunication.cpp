// methodology adapted from https://github.com/dmicha16/simple_serial_port
// meant for use in MOAD data collection rig

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

#include "SerialCommunication.h"
#include "DebugUtils.h"

SimpleSerial::SimpleSerial(const char* portname, int baudrate) {

    // open serial port (file since everything in linux is a file)
    //  note: somewhere someone suggested O_NDELAY but this causes a block somewhere
    fd_ = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK); // args are read/write, no controlling terminal, non-blocking
    if (fd_ == -1) {
        std::cerr << "Error opening serial port " << portname << ": " << strerror(errno) << std::endl;
        connected_ = false;
        return;
    }

    // Clear non-blocking flag so reads block until data available
    fcntl(fd_, F_SETFL, 0); 

    // configure port:
    struct termios options;
    if (tcgetattr(fd_, &options) != 0) {
        std::cerr << "Error getting port attributes: " << strerror(errno) << std::endl;
        close(fd_);
        connected_ = false;
        return;
    }

    // set baud rates:
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // below settings are "8N1 framing: 8 data bits, no parity, 1 stop bit"
    //  supposedly common for pc to arduino. personally not sure how they work
    options.c_cflag &= ~PARENB;   // No parity
    options.c_cflag &= ~CSTOPB;   // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;       // 8 bits
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem control lines

    // Raw input/output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // Apply settings
    tcsetattr(fd_, TCSANOW, &options);

    std::cout << "Serial port " << portname << " opened and configured." << std::endl;
    connected_ = true;

    // flush any stray input/output so we start with a clean buffer
    tcflush(fd_, TCIOFLUSH);

    // slight delay to allow device to reset/respond (e.g., Arduino auto-reset)
    std::cout << "waiting 2 seconds for full arduino startup" << std::endl;
    sleep(2);
}



// called in MOADCui.cpp with "std::string incoming = Serial->ReadSerialPort(wait_time, "json");"

//  so the syntax_type is json but this is not necessary unless the arduino returns json related items
//   but i'll keep it here so we can just swap the header files in the MOADCui.cpp file and not have 
//   to change the implementation

//  wait time is calculated in MOADCui.cpp right before calling this function so we do not need
//   to calculate it here
std::string SimpleSerial::ReadSerialPort(int reply_wait_time, const std::string& syntax_type) {

    // methodology:
    //  each loop iteration the arduino returns a couple characters. we place these into buf[256]
    //  and append to the end of "line". message is then built by placing the lines together.

    std::string line;
    char buf[256];
    std::string message;
    
    auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(reply_wait_time);

    while (true) {

        // setup timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100 ms polling interval

        // setup file descriptor set with serial port fd
        fd_set readfds;
        FD_ZERO(&readfds); // initialize set to empty
        FD_SET(fd_, &readfds); // add serial port fd to set

        // apparently, select() searches all fds from 0 to [the fd we want] + 1 (first arg)
        //  args 2,3,4 are meant to be sets to CHECK FOR READINESS, in this case we cheack the read set
        //  tv is the timeout to wait
        int rv = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);

        // now check result of select()
        if (rv < 0) {
            perror("select has failed");
            break;
        } else if (rv == 0) {
            // No data ready, continue
            // std::cout << "No data within 100 ms." << std::endl;
        } else if (FD_ISSET(fd_, &readfds)) {
            // data can be read
            
            // read from the serial port
            //  break if read fails
            //  continue if no data read
            int n = read(fd_, buf, sizeof(buf) - 1);
            if (n < 0) { perror("read has failed"); break; }
            if (n == 0) continue;

            // null-terminate and append to line buffer
            buf[n] = '\0';
            line += buf;


            // process complete lines by finding newline characters
            //  just use substrings via find() and substr()
            size_t pos;
            while ((pos = line.find('\n')) != std::string::npos) {
                message += line.substr(0, pos) + " ";

                // remove processed line from buffer
                line.erase(0, pos + 1);
                
                // std::cout << "Received complete: " << message << std::endl;
            }
        }

        if (message.find("}") != std::string::npos) {
            DebugUtils::logSerial("Termination string \"}\" received.");
            // std::cout << "Termination string \"}\" received." << std::endl;
            break;
        }

        // check for timeout
        if (std::chrono::steady_clock::now() - start > timeout) {
            DebugUtils::logSerial("Read timeout ("+std::to_string(reply_wait_time)+" seconds) exceeded.");
            // std::cout << "Read timeout exceeded." << std::endl;
            break;
        }
    }

    return message;
}


bool SimpleSerial::WriteSerialPort(const char *data_sent) {

    std::string data = data_sent;
    DebugUtils::logSerial("Message Received: " + data);
    // std::cout << data_sent << std::endl;
    // std::cout << fd_ << std::endl;

    size_t len = strlen(data_sent);
    ssize_t n = write(fd_, data_sent, len);

    // flush output after writing
    tcdrain(fd_);

    if (n < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}


// close and delete:
bool SimpleSerial::CloseSerialPort() {
    if (connected_) {
        close(fd_);
        connected_ = false;
        std::cout << "Serial port closed." << std::endl;
        return true;
    }
    return false;
}

SimpleSerial::~SimpleSerial() {
    CloseSerialPort();
}

/*
  in MOADCui.cpp (main) we import <SimpleSerial.h>
    then we use:

    rotate_turntable():
    - init:
        SimpleSerial* Serial;
    - WriteSerialPort
    	bool is_sent = Serial->WriteSerialPort(send);
    - ReadSerialPort
		std::string incoming = Serial->ReadSerialPort(wait_time, "json");

    turntableControl():
    - WriteSerialPort
    	bool is_sent = Serial->WriteSerialPort(send);
    - ReadSerialPort
        std::string incoming = Serial->ReadSerialPort(wait_time, "json");

    main():
    constructor:

    Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
	if(Serial->connected_) {
		cout << "Serial connected to " << com_port << endl;
	} else {
		cout << "Error creating serial connection.\n";
	}
*/
