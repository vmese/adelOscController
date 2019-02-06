#ifndef arbotixController_H
#define arbotixController_H

#include "ofMain.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "boost/signals2.hpp"

#include "ofArbotix.h"

#define TC_ID 15
#define CF_ID 1
#define GRIP_ID 1
#define ax12TempRegister 0x2B
#define ax12PosRegister 0x24
#define ax12LBLoadRegister 0x28
#define ax12HBLoadRegister 0x29
#define ax12PGainRegister 0x1C

class arbotixController
{
protected:

    boost::signals2::connection m_EInitializedConnection;
    boost::signals2::connection m_EDigitalPinChanged;
    boost::signals2::connection m_EAnalogPinChanged;
    boost::signals2::connection m_ECommanderChanged;
    boost::signals2::connection m_EDynamixelKeyReceived;
    boost::signals2::connection m_EDynamixelAllReceived;
    boost::signals2::connection m_EDynamixelTransmitError;
    boost::signals2::connection m_EDynamixelGetRegister;

    bool fbSwingLeg;

    void digitalPinChanged(const int & pinNum);
    void analogPinChanged(const int & pinNum);
    void commanderChanged(const int & pinNum);
    void dynamixelRecieved(const int & servoNum);
    void dynamixelTransmitError(const int & cmd, const int & servoNum);
    void dynamixelGetRegister(const unsigned int &servo, const unsigned int &reg, const unsigned int &value);

#ifdef INCLUDE_TIMING
    unsigned long long m_lStartTick;
    unsigned long long m_lEndTick;
    double m_dblTotalMillis;
    long m_lTotalCount;
#endif

public:
    //arbotixController(const std::string &name, const int &rate);
    arbotixController();
    void initialize(const int &version);
    void update();
    void connectController(const string &portName, int rate);
    void disconnect();
    static arbotixController* getInstance();
    static void initializeCallback(const int &version);
    bool isInitialized();
    void setup();
    //void loadConfiguration(const std::string &fileName);
    void sendServoAngle(int servoId,int angle, int speed);
    void attachServo(const uint &servoId);
    //void resetServo(const uint &servoId);
    int getServoTemp(const unsigned int &servoId);
    int getServoPos(const unsigned int &servoId);
    int getServoLoad(const unsigned int &servoId);
    int getServoPGain(const unsigned int &servoId);
    void enableServo(const unsigned int &servoId);
    void disableServo(const unsigned int &servoId);
    void getDynamixelRegister(const unsigned char &servo, const unsigned char &reg, const unsigned char &length);
    void setDynamixelRegister(const unsigned char &servo, const unsigned char &reg, const unsigned char &length, const unsigned int &value);
    void moveServos();
    bool moveServoAndWait(const int & servoId, const int & pos, const int & speed );
    bool waitForDynamixelStopped(unsigned char servo);
    void test();
    void sendMsgToPic(const char picId, const char picCmd, const char *picMess);
    void setPGain(const unsigned int & servoId, const unsigned int &pGain);

private :

    void xConnect(const std::string &portName, const int rate);

    ofArbotix *arbotix;
    bool xInitialized;
    static arbotixController *fInstance;
    ofEvent <const int> arduinoInitiliazedEvent;

    boost::mutex fMutex;
    std::map<int,int> fServosTemps;
    std::map<int,int> fServosPos;
    std::map<int,int> fLBServosLoad;
    std::map<int,int> fHBServosLoad;
    std::map<int,int> fPGainServos;


};

#endif // arbotixController_H
