#include "arbotixController.h"


arbotixController * arbotixController::fInstance = 0;


void arbotixController::initializeCallback(const int &version)
{
    ofLogNotice() << "Entering into init callback";
    arbotixController *tmp = arbotixController::getInstance();
    tmp->initialize(version);
}

arbotixController::arbotixController():
xInitialized(false),
fbSwingLeg(false)
{
  printf("arbotixController::arbotixController\n");
  fInstance = this;
  xInitialized = false;
  arbotix = new ofArbotix();
  if (arbotix == NULL)
  {
      ofLogError("Error while contructing arduino");
  }


#ifdef INCLUDE_TIMING
    m_lStartTick= 0;
    m_lEndTick= 0;
    m_lTotalCount= -1;
    m_dblTotalMillis = 0;
#endif
}

void arbotixController::connectController(const std::string &portName, int rate)
{
    printf("arbotixController::connect\n");
    if (portName == "" || rate == 0)
    {
        ofLogError() << "name of the port and baud rate not specified";
    }

    ofLogNotice() << "Try to connect to " << portName << " at " << rate;
    bool connected = arbotix->connect(portName,rate);

    if (connected)
    {
        ofLogNotice("success");
    }

    //arbotix->sendReset();
    //arbotix->sendProtocolVersionRequest();
    arbotix->sendFirmwareVersionRequest();
    printf("Wait for ready Event\n");
    m_EInitializedConnection = arbotix->EInitialized.connect(boost::bind(&arbotixController::initialize, this, _1));
    //ofAddListener(arbotix->EInitialized,&initializeCallback);
}

void arbotixController::initialize(const int &version)
{
    //boost::mutex::scoped_lock lock(fMutex);
    ofLogNotice() << "Entering into init";
    m_EInitializedConnection.disconnect();
    m_EDigitalPinChanged = arbotix->EDigitalPinChanged.connect(boost::bind(&arbotixController::digitalPinChanged, this, _1));
    m_EAnalogPinChanged = arbotix->EAnalogPinChanged.connect(boost::bind(&arbotixController::analogPinChanged, this, _1));
    m_ECommanderChanged = arbotix->ECommanderDataReceived.connect(boost::bind(&arbotixController::commanderChanged, this, _1));
    m_EDynamixelKeyReceived = arbotix->EDynamixelKeyReceived.connect(boost::bind(&arbotixController::dynamixelRecieved, this, _1));
    m_EDynamixelAllReceived = arbotix->EDynamixelAllReceived.connect(boost::bind(&arbotixController::dynamixelRecieved, this, _1));
    m_EDynamixelTransmitError = arbotix->EDynamixelTransmitError.connect(boost::bind(&arbotixController::dynamixelTransmitError, this, _1, _2));
    m_EDynamixelGetRegister = arbotix->EDynamixelGetRegister.connect(boost::bind(&arbotixController::dynamixelGetRegister, this, _1, _2, _3));

    //ofRemoveListener(arbotix->EInitialized,&initializeCallback);
    printf("getFirmwareNames\n");
    ofLogNotice() << arbotix->getFirmwareName();
    ofLogNotice() << "firmata v" << arbotix->getMajorFirmwareVersion() << "." << arbotix->getMinorFirmwareVersion();


    xInitialized = true;

    setup();

    printf("done\n");

}


void arbotixController::attachServo(const uint &servoId)
{
    if (xInitialized)
    {
        arbotix->sendDynamixelServoAttach(servoId);
    }

}

bool arbotixController::waitForDynamixelStopped(unsigned char servo)
{
    arbotix->sendDynamixelStopped(servo);
    int ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_STOPPED, 10);
    return ret;
}

//void arbotixController::resetServo(const uint &servoId)
//{
//    if (xInitialized)
//    {
//        arbotix->resetServo(servoId);
//    }

//}

void arbotixController::setup()
{
    //boost::mutex::scoped_lock lock(fMutex);
    printf("arbotixController::setup()\n");
    if (xInitialized)
    {
//        for (int i=0;i<fServosNames.size();i++)
//        {
//            arbotix->sendServoAttach(fServosPins[i]);
//            arbotix->sendServo(fServosPins[i], fServosInitialPositions[i], false);
//        }


        //Set the CW and CCW limits to be in a valid range for this test
//        arbotix->sendDynamixelSetRegister(4, 0x19, 2, 1);
//        arbotix->sendDynamixelSetRegister(TC_ID, 0x08, 2, 810);

//        arbotix->sendDynamixelGetRegister(4, 0x19, 2);
//        bool ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER, 2);
//        arbotix->sendDynamixelGetRegister(TC_ID, 0x08, 2);
//        ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER, 2);
//        ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_SET_REGISTER, 2);

//        arbotix->sendDynamixelConfigureServo(TC_ID, 170, 800, 1023, 1, 1, 1, 32, 32);

//        //Test waiting for motor to stop
//        arbotix->sendDynamixelMove(TC_ID, 512, 0);
//        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

//        arbotix->sendDynamixelMove(TC_ID, 200, 40);
//        arbotix->sendDynamixelStopped(TC_ID);
//        ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_STOPPED, 10);

//        arbotix->sendDynamixelServoAttach(TC_ID);
//        arbotix->sendDynamixelServoAttach(CF_ID);
//        arbotix->sendDynamixelServoAttach(GRIP_ID);

//        arbotix->sendDynamixelMove(GRIP_ID, 400, 0);
//        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

//        arbotix->sendDynamixelMove(CF_ID, 400, 0);
//        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

//        arbotix->sendDynamixelMove(TC_ID, 400, 0);
//        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

//        //Start a move then stop it.
//        arbotix->sendDynamixelMove(TC_ID, 700, 20);
//        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

//        arbotix->sendDynamixelStop(TC_ID);

//        //Wait for a second.
//        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//        fbSwingLeg = false;
//        arbotix->sendDynamixelSynchMoveAdd(TC_ID, 512, 100);
//        arbotix->sendDynamixelSynchMoveAdd(CF_ID, 512, 100);
//        arbotix->sendDynamixelSynchMoveAdd(GRIP_ID, 512, 100);
//        arbotix->sendDynamixelSynchMoveExecute();

//        //Wait for a second.
//        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//        //Get the motor moving slowly over to the side
//        arbotix->sendDynamixelSynchMoveAdd(TC_ID, 180, 100);
//        arbotix->sendDynamixelSynchMoveAdd(CF_ID, 650, 100);
//        arbotix->sendDynamixelSynchMoveAdd(GRIP_ID, 650, 100);
//        arbotix->sendDynamixelSynchMoveExecute();
//        fbSwingLeg = true;


    }
    printf("ArbotixController : setup done\n");
}

void arbotixController::sendServoAngle(int servoId, int angle, int speed)
{
    //arbotix->sendServo(pinNb, angle, false);
    //int angleValue = (int) angle*1024/300;

    if (angle<=0 || angle >1024)
    {
        //ofLogError() << "Error : angle not correct : " << angle;
    }
    else if (servoId<0 || servoId >4)
    {
        ofLogError() << "Error : servoId not correct : " << servoId;
    }
    else
    {
        //printf("set servo angle : ID - %i - Angle %i - Speed %i\n",servoId,angle,speed);
        //arbotix->sendDynamixelMove(servoId, angle, speed);
        arbotix->sendDynamixelSynchMoveAdd(servoId, angle, speed);
    }

}

int arbotixController::getServoTemp(const unsigned int &servoId)
{
    getDynamixelRegister(servoId,ax12TempRegister,2);
    std::map <int,int>::iterator it;
    it=fServosTemps.find(servoId);
    if (it!=fServosTemps.end())
    {
        return fServosTemps[servoId];
    }
    else
    {
        return 0;
    }
}

int arbotixController::getServoPos(const unsigned int &servoId)
{
    getDynamixelRegister(servoId,ax12PosRegister,2);
    std::map <int,int>::iterator it;
    it=fServosPos.find(servoId);
    if (it!=fServosPos.end())
    {
        return fServosPos[servoId];
    }
    else
    {
        return 0;
    }
}

void arbotixController::moveServos()
{
    arbotix->sendDynamixelSynchMoveExecute();
}

bool arbotixController::moveServoAndWait(const int & servoId, const int & pos, const int & speed )
{
    arbotix->sendDynamixelMove(servoId,pos,speed);
    int ret = waitForDynamixelStopped(servoId);
    return ret;
}

void arbotixController::disconnect()
{
   arbotix->disconnect();
   xInitialized = false;

}


void arbotixController::test()
{
    arbotix->sendDynamixelMove(0, 512, 256);
    arbotix->sendDynamixelStopped(0);
    int ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_STOPPED, 10);
    printf("ret = %i\n",ret);
}

void arbotixController::enableServo(const unsigned int &servoId)
{

}

void arbotixController::disableServo(const unsigned int &servoId)
{
    printf("abotix : disable servo %i\n",servoId);
    //arbotix->sendDynamixelStop(servoId);
    arbotix->sendDynamixelSetRegister(servoId, 0x18, 2, 0);
    //bool ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_SET_REGISTER, 2);
    //printf("ret : %i\n",ret);

}


void arbotixController::update()
{

    arbotix->update();
}

arbotixController *arbotixController::getInstance()
{
    return fInstance;
}

bool arbotixController::isInitialized()
{
    return xInitialized;
}

//void arbotixController::loadConfiguration(const std::string &fileName)
//{
//    std::string message;
//    if( fXMLReader.loadFile(fileName) ){
//        message = fileName + " loaded!";
//        ofLogNotice() << message;
//    }else{
//        message = "unable to load " + fileName + " - check data/ folder";
//        ofLogError() << message;
//    }

//    // find port name and baudrate
//    std::string portName = fXMLReader.getValue("port::name", "");
//    int rate = fXMLReader.getValue("port::rate", 0);

//    if (portName == "" || rate == 0)
//    {
//        ofLogError() << "name of the port and baud rate not find in xml";
//    }
//    else
//    {
//        fPortName = portName ;
//        fRate = rate;
//    }

//    // find servos names
//    fServosNames.clear();
//    fServosPins.clear();
//    fServosIds.clear();
//    fServosInitialPositions.clear();

//    int nbServos = fXMLReader.getNumTags("servo");
//    if (nbServos>0)
//    {
//        for(int i = 0; i < nbServos; i++)
//        {
//              fXMLReader.pushTag("servo", i);
//              std::string servoName = fXMLReader.getValue("name","");
//              int id = fXMLReader.getValue("id",0);
//              int pinNb = fXMLReader.getValue("pinNb",0);
//              int pos = fXMLReader.getValue("initialPos",0);
//              ofLogNotice () << "Servo " << servoName << " id : " <<  id << " attached to pin " << pinNb;

//              if (servoName!="" && pinNb!=0)
//              {
//                  fServosNames.push_back(servoName);
//                  fServosPins.push_back(pinNb);
//                  fServosIds.push_back(id);
//                  fServosInitialPositions.push_back(pos);
//              }
//              fXMLReader.popTag();
//         }

//    }
//}

void arbotixController::xConnect(const std::string &portName, const int rate)
{


}

//--------------------------------------------------------------
void arbotixController::digitalPinChanged(const int & pinNum)
{
    // do something with the digital input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    int iVal = arbotix->getDigital(pinNum);
    //std::cout << "digital pin: " << pinNum << " = " << iVal << "\r\n";

    //arbotix->sendDigital(13, iVal);
}

// analog pin event handler, called whenever an analog pin value has changed

//--------------------------------------------------------------
void arbotixController::analogPinChanged(const int & pinNum)
{
    // do something with the analog input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    int iVal = arbotix->getAnalog(pinNum);
    std::cout << "analog pin: " << pinNum << " = " << iVal << "\r\n";
}

//commander event handler, called whenever a commander button value has changed

//--------------------------------------------------------------
void arbotixController::commanderChanged(const int & pinNum)
{
 //   // do something with the analog input. here we're simply going to print the pin number and
 //   // value to the screen each time it changes
    //int iVal = arbotix->getAnalog(pinNum);
    std::cout << "Commander changed: buttons" << arbotix->_commanderData._buttons <<
        " walkV " << (int) arbotix->_commanderData._walkV <<
        " walkH " << (int) arbotix->_commanderData._walkH <<
        " lookV " << (int) arbotix->_commanderData._lookV <<
        " lookH " << (int) arbotix->_commanderData._lookH <<
        " buttons " << (int) arbotix->_commanderData._buttons << "\r\n";
}

//dynamixel event handler, called whenever a dynamixel update occurs

//--------------------------------------------------------------
void arbotixController::dynamixelRecieved(const int & servo)
{
//#ifdef INCLUDE_TIMING
//    if(m_lTotalCount >= 0)
//    {
//        m_lEndTick = osg::Timer::instance()->tick();

//        double dblMillis = osg::Timer::instance()->delta_m(m_lStartTick, m_lEndTick);

//        m_dblTotalMillis+=dblMillis;
//        m_lTotalCount++;

//        if(m_lTotalCount >= 100)
//        {
//            double dblAvg = m_dblTotalMillis/m_lTotalCount;
//            //std::cout << "Millis: " << dblAvg << ", Total: " << m_lTotalCount << "\r\n";
//            std::cout << "Avg: " << dblAvg << "\r\n";
//            m_lTotalCount = 0;
//            m_dblTotalMillis= 0;
//        }
//    }
//#endif

// //   // do something with the analog input. here we're simply going to print the pin number and
// //   // value to the screen each time it changes
//    //if(servo == TC_ID)
//    //    std::cout << "servo: " << servo << ", Pos: " << _dynamixelServos[servo]._actualPosition << ", swing: " << fbSwingLeg  << "\r\n";

//    arbotix->_dynamixelServos[servo]._keyChanged = false;
//    arbotix->_dynamixelServos[servo]._allChanged = false;

//    if(	fbSwingLeg && servo == TC_ID && arbotix->_dynamixelServos[servo]._actualPosition <= (185))
//    {
//        fbSwingLeg = false;
//        arbotix->sendDynamixelSynchMoveAdd(TC_ID, 800, 100);
//        arbotix->sendDynamixelSynchMoveAdd(CF_ID, 512, 100);
//        arbotix->sendDynamixelSynchMoveAdd(GRIP_ID, 512, 100);
//        arbotix->sendDynamixelSynchMoveExecute();
//        std::cout << "Swing\r\n";
//    }
//    else if(!fbSwingLeg && servo == TC_ID && arbotix->_dynamixelServos[servo]._actualPosition >= (795))
//    {
//        fbSwingLeg = true;
//        arbotix->sendDynamixelSynchMoveAdd(TC_ID, 180, 100);
//        arbotix->sendDynamixelSynchMoveAdd(CF_ID, 650, 100);
//        arbotix->sendDynamixelSynchMoveAdd(GRIP_ID, 650, 100);
//        arbotix->sendDynamixelSynchMoveExecute();
//        std::cout << "Stance\r\n";
//    }

//    //arbotix->sendDynamixelServoDetach(servo);

//#ifdef INCLUDE_TIMING
//    m_lStartTick = osg::Timer::instance()->tick();
//    if(m_lTotalCount<0)
//        m_lTotalCount = 0;
//#endif
}


void arbotixController::dynamixelTransmitError(const int & cmd, const int & servoNum) {

    std::cout << "Transmit error Cmd: " << cmd << ", servo: " << servoNum << "\r\n";
}


void arbotixController::getDynamixelRegister(const unsigned char &servo, const unsigned char &reg, const unsigned char &length)
{
    arbotix->sendDynamixelGetRegister(servo, reg, length);
    bool ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER, length);
    //printf("arbotixController::getDynamixelRegister - ret =%i\n",ret);
}

void arbotixController::setDynamixelRegister(const unsigned char &servo, const unsigned char &reg, const unsigned char &length, const unsigned int &value)
{
    arbotix->sendDynamixelSetRegister(servo, reg, length, value);
}

void arbotixController::dynamixelGetRegister(const unsigned int &servo, const unsigned int &reg, const unsigned int &value) {

    if (reg==ax12TempRegister)
    {
        fServosTemps[servo]=value;
        //ofLogNotice() << "Température Servo " << servo << " : " << value <<  "°C";
    }
    if (reg==ax12PosRegister)
    {
        fServosPos[servo]=value;
        //ofLogNotice() << "Température Servo " << servo << " : " << value <<  "°C";
    }
    if (reg==0x18)
    {
        //fServosPos[servo]=value;
        ofLogNotice() << "Servo " << servo << " : Torque Enbled : " << value ;
    }
    //std::cout << "Get Register Servo: " << servo << ", reg: " << reg << ", value: " << value << "\r\n";
}
