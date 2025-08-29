#include <exception>
#include <string>
#include <sstream>

class CameraException : public std::exception {
private:
    std::string message;

public:
    CameraException() {}
    CameraException(const char* msg) : message(msg) {}
    const char* what() const throw() {
        std::stringstream ss;
        ss << "Camera Exception: \n" << message.c_str(); 
        std::string msgstr = ss.str();
        return "Camera Exception: \n";
    }
};

class CameraObjectNotReadyException : public CameraException {
private:
    std::string message;

public:
    CameraObjectNotReadyException(const char* msg) : message(msg) {}
    const char* what() const throw() {
        std::stringstream ss;
        ss << "Camera Exception: Camera Object Not Ready \n" << message.c_str();
        std::string msgstr = ss.str(); 
        return "Camera Exception: Camera Object Not Ready \n";
    }
};

class CameraBusyException : public CameraException {
private:
    std::string message;

public:
    CameraBusyException(const char* msg) : message(msg) {}
    const char* what() const throw() {
        std::stringstream ss;
        ss << "Camera Exception: Camera Busy \n" << message.c_str(); 
        std::string msgstr = ss.str();
        return "Camera Exception: Camera Busy \n";
    }
};