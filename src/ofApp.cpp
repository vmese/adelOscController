#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetVerticalSync(true);
    ofSetFrameRate(30);
    //ofBackground(0,0,0); // black
    ofBackground(255,255,255); // white

    // window elements
    verdana14Font.load("verdana.ttf", 14, true, true);
    verdana14Font.setLineHeight(18.0f);
    verdana14Font.setLetterSpacing(1.037);


    // setup gui from ofxGui
    _gui.setup("Gui");
    _gui.setPosition(0 , 0);
    _gui.minimizeAll();


    fTrackHead = false;
    //fMotorsEnabled = false;

    arbotix = new arbotixController();
    loadConfiguration("arbotixConfig.xml");


    for (int i=0;i<kNbOfServos;i++)
    {
        //arbotix->attachServo(i);
    }

    printf("setup servos % i %i",fServosNames.size(),fServosIds.size());


    servo1.setController(arbotix);
    servo1.setName("servo1");
    servo1.setId(1);
    servo1.setSpeed(256); // 256

    servo2.setController(arbotix);
    servo2.setName("servo2");
    servo2.setId(2);
    servo2.setSpeed(50); //50

    servo3.setController(arbotix);
    servo3.setName("servo3");
    servo3.setId(3);
    servo3.setSpeed(85); //85

    servo4.setController(arbotix);
    servo4.setName("servo4");
    servo4.setId(4);
    servo4.setSpeed(128);

    servo5.setController(arbotix);
    servo5.setName("servo5");
    servo5.setId(5);
    servo5.setSpeed(128);

    printf("setup servos done");


    ///setup osc connexion and controls
    printf("setup osc connexion and controls");

    fAngleControl1.setName("Servo 1");
    fAngleControl1.add(fAngleServo1.set("angle",0.5,0.0,1.0));
    fMinMaxControl1.setName("Min/Max Servo 1");
    fMinMaxControl1.add(fMinServo1.set("Min",fServosMins[0],1.0,1024.0));
    fMinMaxControl1.add(fMaxServo1.set("Max",fServosMax[0],1.0,1024.0));

    fAngleControl2.setName("Servo 2" );
    fAngleControl2.add(fAngleServo2.set("angle",0.5,0.0,1.0));
    fMinMaxControl2.setName("Min/Max Servo 2");
    fMinMaxControl2.add(fMinServo2.set("Min",fServosMins[1],1.0,1024.0));
    fMinMaxControl2.add(fMaxServo2.set("Max",fServosMax[1],1.0,1024.0));

    fAngleControl3.setName("Servo 3");
    fAngleControl3.add(fAngleServo3.set("angle",0.5,0.0,1.0));
    fMinMaxControl3.setName("Min/Max Servo 3");
    fMinMaxControl3.add(fMinServo3.set("Min",fServosMins[2],1.0,1024.0));
    fMinMaxControl3.add(fMaxServo3.set("Max",fServosMax[2],1.0,1024.0));

    fAngleControl4.setName("Servo 4");
    fAngleControl4.add(fAngleServo4.set("angle",0.5,0.0,1.0));
    fMinMaxControl4.setName("Min/Max Servo 4");
    fMinMaxControl4.add(fMinServo4.set("Min",fServosMins[3],1.0,1024.0));
    fMinMaxControl4.add(fMaxServo4.set("Max",fServosMax[3],1.0,1024.0));

    fAngleControl5.setName("Servo 5");
    fAngleControl5.add(fAngleServo5.set("angle",0.5,0.0,1.0));
    fMinMaxControl5.setName("Min/Max Servo 5");
    fMinMaxControl5.add(fMinServo5.set("Min",fServosMins[4],1.0,1024.0));
    fMinMaxControl5.add(fMaxServo5.set("Max",fServosMax[4],1.0,1024.0));

    fHeadPositionControl.setName("Head");
    fHeadPositionControl.add(fHeadPositionX.set("X",512,0,1024));
    fHeadPositionControl.add(fHeadPositionY.set("Y",384,0,768));

    fBooleanControls.setName("Controls");
    fBooleanControls.add(fMotorsEnabled.set("Motors enabled",false));
    fMotorsEnabled.addListener(this,&ofApp::enableMotors);

    fGlobalControls.add(fAngleControl1);
    fGlobalControls.add(fAngleControl2);
    fGlobalControls.add(fAngleControl3);
    fGlobalControls.add(fAngleControl4);
    fGlobalControls.add(fAngleControl5);
    fGlobalControls.add(fHeadPositionControl);
    fGlobalControls.add(fMinMaxControl1);
    fGlobalControls.add(fMinMaxControl2);
    fGlobalControls.add(fMinMaxControl3);
    fGlobalControls.add(fMinMaxControl4);
    fGlobalControls.add(fMinMaxControl5);
    fGlobalControls.add(fBooleanControls);

    _gui.setup(fGlobalControls);
    iccoreConnexion.setup((ofParameterGroup&)_gui.getParameter(),6669,fRemoteControllerIP,fRemoteControllerListeningPort);


    //ofSetLogLevel(OF_LOG_VERBOSE);

    servo1.setup(fServosMins[0], fServosMax[0]);
    servo2.setup(fServosMins[1], fServosMax[1]);
    servo3.setup(fServosMins[2], fServosMax[2]);
    servo4.setup(fServosMins[3], fServosMax[3]);
    servo5.setup(fServosMins[4], fServosMax[4]);

    arbotix->connectController(fArbotixPortName,fArbotixRate);


}




//--------------------------------------------------------------
void ofApp::update(){

    iccoreConnexion.update();

    int elapsedTime = ofGetElapsedTimeMillis();
    //ofLogNotice() << "Elapsed time : " <<  elapsedTime ;

    // check head position;
    if (fTrackHead==true)
    {
        //usleep(500000);

        //take mean of head positions
    //    int sumValX = 0;
    //    int sumValY = 0;

    //    int count =0;
    //    for (int i=0;i<500;i++)
    //    {
    //        sumValX +=fOssiaHeadPositionX;
    //        sumValY +=fOssiaHeadPositionY;

    //        count +=1;
    //    }

        //fmeanHeadPositionX = ofMap(1024-sumValX/count,0,1024,0.,1.);
        //fmeanHeadPositionY = ofMap(768-sumValY/count,0,768,0.,1.);

        //take head positions at each frame
        fmeanHeadPositionX = ofMap(1024-fHeadPositionX,0,1024,0.,1.);
        fmeanHeadPositionY = ofMap(768-fHeadPositionY,0,768,0.,1.);

//        fmeanHeadPositionX = ofMap(fOssiaHeadPositionX,0,1024,0.,1.);
//        fmeanHeadPositionY = ofMap(768-fOssiaHeadPositionY,0,768,0.,1.);


        fAngleServo4.set(fmeanHeadPositionY);
        fAngleServo5.set(fmeanHeadPositionX);
        printf("X HEAD : %f\n",fmeanHeadPositionX);
        printf("Y HEAD : %f\n",fmeanHeadPositionY);
    }


    // set or get servos angles

    if (fMotorsEnabled==false)
    {
        int posServo1 = servo1.getPos();
        int posServo2 = servo2.getPos();
        int posServo3 = servo3.getPos();
        int posServo4 = servo4.getPos();
        int posServo5 = servo5.getPos();

        float pos1 = ofMap(posServo1, fServosMins[0], fServosMax[0],0.,1.);
        float pos2 = ofMap(posServo2, fServosMins[1], fServosMax[1],0.,1.);
        float pos3 = ofMap(posServo3, fServosMins[2], fServosMax[2],0.,1.);
        float pos4 = ofMap(posServo4, fServosMins[3], fServosMax[3],0.,1.);
        float pos5 = ofMap(posServo5, fServosMins[4], fServosMax[4],0.,1.);

        
        fAngleServo1.set(pos1);
        fAngleServo2.set(pos2);
        fAngleServo3.set(pos3);
        fAngleServo4.set(pos4);
        fAngleServo5.set(pos5);
	
        int timeSleepMs = 300;
        usleep(timeSleepMs*1000);
    }



//    servo1.setAngle(fOssiaAngleServo1);
//    servo2.setAngle(fOssiaAngleServo2);
//    servo3.setAngle(fOssiaAngleServo3);
//    servo4.setAngle(fOssiaAngleServo4);
//    servo5.setAngle(fOssiaAngleServo5);

    //servo2.setAngle(fOssiaAngleServo2);

    //Rq : need to be called after arbotix->connect
    arbotix->update();

    //update servos
    if (arbotix->isInitialized() && fMotorsEnabled) {
        //printf("set servo 2 angle to %f\n",fAngleServo2.get());
        servo1.setup(fMinServo1,fMaxServo1);
        servo2.setup(fMinServo2,fMaxServo2);
        servo3.setup(fMinServo3,fMaxServo3);
        servo4.setup(fMinServo4,fMaxServo4);
        servo5.setup(fMinServo5,fMaxServo5);

        servo1.setAngle(fAngleServo1.get());
        servo2.setAngle(fAngleServo2.get());
        servo3.setAngle(fAngleServo3.get());
        servo4.setAngle(fAngleServo4.get());
        servo5.setAngle(fAngleServo5.get());
        servo1.update();
        servo2.update();
        servo3.update();
        servo4.update();
        servo5.update();
        arbotix->moveServos();
     }


    // check servos parameters
    //int tempServo3 = arbotix->getServoTemp(3);
    //printf ("temp servo 3 :%i\n",tempServo3);

    //bool ret = arbotix->waitForSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER, 2);


     // check servos temp

     if (elapsedTime>=2000)
     {
         // read a first value, otherwise temp is false in second reading (0x2B)
        arbotix->getDynamixelRegister(3,0x24,2);

        fServo2Temp = servo2.getTemp();
        fServo3Temp = servo3.getTemp();
        //printf ("Temp Servo 2 = %i °C\n",fServo2Temp);
        //printf ("Temp Servo 3 = %i °C\n",fServo3Temp);

        ofResetElapsedTimeCounter() ;
     }

    //arbotix->getDynamixelRegister(4,0x2B,2);
    //arbotix->getDynamixelRegister(5,0x2B,2);

    //usleep(100000);
    //ofResetElapsedTimeCounter() ;

}

void ofApp::standUp()
{
     fAngleServo3.set(1.0);
//     int timeSleepS = 1;
//     usleep(timeSleepS*1000000);
     fAngleServo2.set(1.0);
     fAngleServo4.set(0.8);
     fAngleServo5.set(0.5);


}

void ofApp::goToRest()
{
      fTrackHead = false;

}



//--------------------------------------------------------------
void ofApp::draw(){
 _gui.draw();

 // display temp servo 2
 ofSetColor(ofColor::black);
 if (fServo2Temp>=60 and fServo2Temp<=70)
 {
    ofSetColor(ofColor::orange);
 }
 else if (fServo2Temp>=70)
 {
     ofSetColor(ofColor::red);
 }
 verdana14Font.drawString("Temp Servo 2 :" + ofToString(fServo2Temp) + "°C",ofGetWindowWidth()- 200,150);


 // display temp servo 3

 ofSetColor(ofColor::black);
 if (fServo3Temp>=60 and fServo3Temp<=70)
 {
    ofSetColor(ofColor::orange);
 }
 else if (fServo3Temp>=70)
 {
     ofSetColor(ofColor::red);
 }
 verdana14Font.drawString("Temp Servo 3 :" + ofToString(fServo3Temp) + "°C",ofGetWindowWidth()- 200,200);


 ofDrawBitmapString(ofToString((int) ofGetFrameRate()), ofGetWidth() - 20, ofGetHeight() - 10);
 ofDrawBitmapStringHighlight(
         string() +
         "z - repose-toi\n" +
         "s - leve toi\n" +
         "a - asservissement moteurs\n" +
         "t - suivi visage\n"
         ,ofGetWindowWidth()- 200, 250);
}

void ofApp::enableMotors(bool &state)
{
    if (state==true)
    {
        fMotorsEnabled = true;
    }
    else
    {
        servo3.disable();
        servo2.disable();
        servo4.disable();
        servo5.disable();
        servo1.disable();
        fMotorsEnabled = false;
    }
}

//void ofApp::enableMotors(bool state)
//{
//    if (state==true)
//    {
//        fMotorsEnabled = true;
//    }
//    else
//    {
//        servo3.disable();
//        servo2.disable();
//        servo4.disable();
//        servo5.disable();
//        servo1.disable();
//        fMotorsEnabled = false;
//    }
//}
//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    switch(key){

    case 'a':
        if (fMotorsEnabled ==false)
        {
            bool val = true;
            enableMotors(val);

        }
        else if (fMotorsEnabled==true)
        {
            bool val = false;
            enableMotors(val);
        }
        break;


    case 't' :
        if (fTrackHead ==false)
        {
            fTrackHead = true;
        }
        else if (fTrackHead==true)
        {
            fTrackHead = false;
        }
        break;

    case 's' :
        standUp();
        break;

    case 'r' :
        goToRest();
        break;

    case 'l' :
        //arbotix->setDynamixelRegister(4,0x19,2,1);
        break;


    default:
        break;
    }


}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::exit()
{
    ofLogNotice() << "Exit app";
    arbotix->disconnect();
}


void ofApp::loadConfiguration(const std::string &fileName)
{
    std::string message;
    if( fXMLReader.loadFile(fileName) ){
        message = fileName + " loaded!";
        ofLogNotice() << message;
    }else{
        message = "unable to load " + fileName + " - check data/ folder";
        ofLogError() << message;
    }

    // find port name and baudrate
    std::string portName = fXMLReader.getValue("port::name", "");
    int rate = fXMLReader.getValue("port::rate", 0);

    if (portName == "" || rate == 0)
    {
        ofLogError() << "name of the port and baud rate not find in xml";
    }
    else
    {
        fArbotixPortName = portName ;
        fArbotixRate = rate;
    }

    // find address and port for Osc communication
    std::string remoteIP= fXMLReader.getValue("oscParams::remoteIP", "");
    int port = fXMLReader.getValue("oscParams::listeningPort", 0);

    if (remoteIP == "" || port == 0)
    {
        ofLogError() << "no IP address and port found for Osc communication";
    }
    else
    {
        fRemoteControllerIP = remoteIP ;
        fRemoteControllerListeningPort = port;
    }

    // find servos names
    fServosList.clear();
    fServosNames.clear();
    fServosIds.clear();
    fServosMins.clear();
    fServosMax.clear();
    fServosInitialPos.clear();

    int nbServos = fXMLReader.getNumTags("servo");
    if (nbServos>0)
    {
        for(int i = 0; i < nbServos; i++)
        {
              fXMLReader.pushTag("servo", i);
              std::string servoName = fXMLReader.getValue("name","");
              int id = fXMLReader.getValue("id",0);
              int pinNb = fXMLReader.getValue("pinNb",0);
              int pos = fXMLReader.getValue("initialPos",0);
              int min = fXMLReader.getValue("min",0);
              int max = fXMLReader.getValue("max",300);

              ofLogNotice () << "Servo " << servoName << " id : " <<  id << " initial pos " << pos;

              if (servoName!="" && id!=0)
              {
                  ofLogNotice () << "Servo " << servoName << " id : " <<  id << " min : " << min;
                  ofLogNotice () << "Servo " << servoName << " id : " <<  id << " max : " << max;
                  fServosNames.push_back(servoName);
                  fServosIds.push_back(id);
                  fServosMins.push_back(min);
                  fServosMax.push_back(max);
                  printf("initial pos : %i\n",pos);
                  fServosInitialPos.push_back(pos);

              }
              fXMLReader.popTag();
         }

    }
}
