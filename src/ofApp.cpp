#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetVerticalSync(true);
    glEnable(GL_DEPTH_TEST);
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

    //setup vision
    w = KW ; //640;
    h = KH ; //480;
    f3DCamera = new camera3D();
    //f3DCamera = new camera3D("127.0.0.1",11999,KW,KH); // remote cam
    f3DImage.create(KH,KW,CV_32FC3);
    bool ret = f3DCamera->start();
    if (ret==false)
    {
        printf("!!! Error while inititializing 3d cam !!!\n");
    }

    // setup the head pose detector
    fHeadPoseDetector = new headPoseDetector();
    if (fHeadPoseDetector!=NULL)
    {
        fHeadPoseDetector->setup();
    }


    arbotix = new arbotixController();
    loadConfiguration("arbotixConfig.xml");
    fbFirstArbotixConnection = true;

    //setup setial com wit arduino
    serialComArduino.listDevices();
    vector <ofSerialDeviceInfo> deviceList = serialComArduino.getDeviceList();

    for (int i=0;i<deviceList.size();i++)
    {
        if (deviceList[i].getDeviceName()=="ttyACM0")
        {
            printf("arduino founded - name = %c \n",deviceList[i].getDeviceName().c_str());
            serialComArduino.setup(deviceList[i].getDeviceID(),9600);
        }
    }


    for (int i=0;i<kNbOfServos;i++)
    {
        //arbotix->attachServo(i);
    }

    printf("---------------setup servos % i %i ---------------",fServosNames.size(),fServosIds.size());


    servo1.setController(arbotix);
    servo1.setName("servo1");
    servo1.setId(0);
    servo1.setSpeed(40); // 256

    servo2.setController(arbotix);
    servo2.setName("servo2");
    servo2.setId(1);
    servo2.setSpeed(50); //50

    servo3.setController(arbotix);
    servo3.setName("servo3");
    servo3.setId(2);
    servo3.setSpeed(58); //85

    servo4.setController(arbotix);
    servo4.setName("servo4");
    servo4.setId(3);
    servo4.setSpeed(128);

    servo5.setController(arbotix);
    servo5.setName("servo5");
    servo5.setId(4);
    servo5.setSpeed(128);

    printf("setup servos done\n");


    ///setup osc connexion and controls
    printf("---------------setup osc connexion and controls-----------------\n");


    fAngleControl1.setName("Servo 1");
    fAngleControl1.add(fAngleServo1.set("angle",0.5,0.0,1.0));
    fMinMaxControl1.setName("Min/Max Servo 1");
    fMinMaxControl1.add(fMinServo1.set("Min",fServosMins[0],1.0,1024.0));
    fMinMaxControl1.add(fMaxServo1.set("Max",fServosMax[0],1.0,1024.0));
    fMinMaxControl1.add(fSpeedServo1.set("Speed",40,40,512));

    fAngleControl2.setName("Servo 2" );
    fAngleControl2.add(fAngleServo2.set("angle",0.5,0.0,1.0));
    fMinMaxControl2.setName("Min/Max Servo 2");
    fMinMaxControl2.add(fMinServo2.set("Min",fServosMins[1],1.0,1024.0));
    fMinMaxControl2.add(fMaxServo2.set("Max",fServosMax[1],1.0,1024.0));
    fMinMaxControl2.add(fServo2Temp.set("Temp",0,20,100));

    fAngleControl3.setName("Servo 3");
    fAngleControl3.add(fAngleServo3.set("angle",0.5,0.0,1.0));
    fMinMaxControl3.setName("Min/Max Servo 3");
    fMinMaxControl3.add(fMinServo3.set("Min",fServosMins[2],1.0,1024.0));
    fMinMaxControl3.add(fMaxServo3.set("Max",fServosMax[2],1.0,1024.0));
    fMinMaxControl3.add(fServo3Temp.set("Temp",0,20,100));

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

    fLedsControl.setName("Leds");
    fLedsControl.add(fRComponentLedValue.set("R",255,0,255));
    fLedsControl.add(fVComponentLedValue.set("V",255,0,255));
    fLedsControl.add(fBComponentLedValue.set("B",255,0,255));
    fLedsControl.add(fBrightnessLedValue.set("Brightness",10,0,255));
    fLedsControl.add(fbEnableExpression.set("Expression On/Off",false));
    fbEnableExpression.addListener(this,&ofApp::enableExpression);
    fLedsControl.add(fLedExpressionValue.set("Expression type",1,1,10));

    fBooleanControls.setName("Controls");
    fBooleanControls.add(fbMotorsEnabled.set("Motors enabled",false));
    fbMotorsEnabled.addListener(this,&ofApp::enableMotors);

    fBooleanControls.add(fbDrawCloud.set("Draw Cloud",false));
    fbDrawCloud.addListener(this,&ofApp::enableDrawCloud);

    fBooleanControls.add(fbFindHead.set("Find head",false));
    fbFindHead.addListener(this,&ofApp::enableFindHead);

    fBooleanControls.add(fbTrackHead.set("Track head",false));
    fbTrackHead.addListener(this,&ofApp::enableHeadTracking);

    servo1.setup(fMinServo1,fMaxServo1);
    servo2.setup(fMinServo2,fMaxServo2);
    servo3.setup(fMinServo3,fMaxServo3);
    servo4.setup(fMinServo4,fMaxServo4);
    servo5.setup(fMinServo5,fMaxServo5);


    fbMotorsEnabled = false;
    fbDrawCloud = false;
    fbFindHead = false;
    fbTrackHead = false;
    fbLaunchBlockingMove = false;
    fbHeadFound = false;

    printf("draw cloud = %i\n",fbDrawCloud);


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
    fGlobalControls.add(fLedsControl);

    _gui.setup(fGlobalControls);
    iccoreConnexion.setup((ofParameterGroup&)_gui.getParameter(),6669,fRemoteControllerIP,fRemoteControllerListeningPort);


    //ofSetLogLevel(OF_LOG_VERBOSE);

    servo1.setup(fServosMins[0], fServosMax[0]);
    servo2.setup(fServosMins[1], fServosMax[1]);
    servo3.setup(fServosMins[2], fServosMax[2]);
    servo4.setup(fServosMins[3], fServosMax[3]);
    servo5.setup(fServosMins[4], fServosMax[4]);

    arbotix->connectController(fArbotixPortName,fArbotixRate);

    cas = 2;
    objectDetectionStartTime = clock();

}




//--------------------------------------------------------------
void ofApp::update(){

    iccoreConnexion.update();

    int elapsedTime = ofGetElapsedTimeMillis();
    //ofLogNotice() << "Elapsed time : " <<  elapsedTime ;



    //--------------------------- VISON --------------------------

    if( cas == 2 && fbDrawCloud == true /*&& elapsedTime>=1000*/)
    {
        f3DCamera->update();
        if (f3DCamera->isFrameNew())
        {
            calcAvgFPS();

            if (fbFindHead == true)
            {
                updateCloud();
                fHeadPoses.clear();
                int nbPoses = fHeadPoseDetector->getHeadPoses(f3DImage,fHeadPoses);

                int currentBrightness = fBrightnessLedValue.get();
                int tmpBrightness;
                if (nbPoses>0)
                {
                    if (fbHeadFound == false)
                    {
                        printf("current brightness = %i\n",currentBrightness);
                        tmpBrightness = currentBrightness+20;
                        if (tmpBrightness>=255)
                        {
                            tmpBrightness = 255;
                        }
                        if (tmpBrightness<=0)
                        {
                            tmpBrightness = 0;
                        }
                        serialComArduino.writeByte('B');
                        serialComArduino.writeByte((char)tmpBrightness);
                        serialComArduino.writeByte('\n');
                        usleep(500000);
                        fBrightnessLedValue.set(currentBrightness);
                        fbHeadFound = true;
                    }
                }
                else
                {
                    if (fbHeadFound == true)
                    {
                        tmpBrightness = currentBrightness-20;
                        if (tmpBrightness>=255)
                        {
                            tmpBrightness = 255;
                        }
                        if (tmpBrightness<=0)
                        {
                            tmpBrightness = 0;
                        }
                        serialComArduino.writeByte('B');
                        serialComArduino.writeByte((char)tmpBrightness);
                        serialComArduino.writeByte('\n');
                        usleep(500000);
                        fBrightnessLedValue.set(currentBrightness);
                        fbHeadFound = false;
                    }
                }



            }
        }

        // check head position;
        if (fbTrackHead==true)
        {
            //take head positions at each frame
            fmeanHeadPositionX = ofMap(1024-fHeadPositionX,0,1024,0.,1.);
            fmeanHeadPositionY = ofMap(768-fHeadPositionY,0,768,0.,1.);

            //printf("Val X : %i\n",1024-fHeadPositionX);
            //printf("Val Y : %i\n",768-fHeadPositionY);


            //printf("X HEAD : %f\n",fmeanHeadPositionX);
            //printf("Y HEAD : %f\n",fmeanHeadPositionY);

            float servo5Angle = ofMap(-fHeadHorizontalPos,-28.5,28.5,0.0,1.0);
            float servo4Angle = ofMap(-fHeadVerticalPos,-21.5,21.5,0.0,1.0);
            fAngleServo4.set(servo4Angle);
            fAngleServo5.set(servo5Angle);

            /* assert arms mouvments */
            if (fHeadVerticalPos>=0)
            {
                float servo3Angle = ofMap(fHeadVerticalPos,0,21.5,0.3,0.8); //0.5 0.8
                fAngleServo3.set(servo3Angle);
                fAngleServo2.set(0.7); //0.5
            }
            else
            {
                float servo2Angle = ofMap(-fHeadVerticalPos,0,21.5,0.7,0.1);
                float servo3Angle = ofMap(-fHeadVerticalPos,0,21.5,0.3,0.0);
                fAngleServo2.set(servo2Angle);
                fAngleServo3.set(servo3Angle);
            }

            // --------- assert base angle on head position (not working) -----------
//            float servo1AngleDegrees = 512 + (fHeadHorizontalPos/180.0*3*512);
//            printf("fHeadHorizontalPos = %f degrees \n ",fHeadHorizontalPos);
//            printf("fAngleServo1 =  %f \n ",fAngleServo1.get());
//            float newServo1Angle = servo1AngleDegrees/1024 - (0.5-fAngleServo1.get()) ;
//            if (newServo1Angle<=0.0)
//            {
//                newServo1Angle = 0.0;
//            }
//            if (newServo1Angle>=1.0)
//            {
//                newServo1Angle = 1.0;
//            }
//            fAngleServo1.set(newServo1Angle);
//            printf("set fAngleServo1 to %f \n ",newServo1Angle);
            // ----------------------------------------------------------------------

            //printf("fAngleServo5 : %f\n",fAngleServo5);
            // if head is too on the left ->rotate bas to the left
            if (elapsedTime>0)
            {
                if (fAngleServo5>=0.95 && fAngleServo1 >=0.1)
                {

                    fbLaunchBlockingMove = true;
                    fbTrackHead = false;
                    fAngleServo1 -= 0.1 ;
                    if (fAngleServo1<=0.0)
                         fAngleServo1=0.0;
                     printf("Decrease base angle to %f\n\n",fAngleServo1);
                    fAngleServo5=0.5;
        //            //fmeanHeadPositionX = 0.5;
                }
                if (fAngleServo5<=0.05 && fAngleServo1 <=0.9 )
                {
                    fbLaunchBlockingMove = true;
                    fbTrackHead = false;
                    fAngleServo1 += 0.1;
                    if (fAngleServo1>=1.0)
                        fAngleServo1=1.0;
                    printf("Increase base angle to %f\n\n",fAngleServo1);
                    fAngleServo5=0.5;
        //            //fmeanHeadPositionX = 0.5;
                }
            } // if elapsedTime>1000
        }
    }

//        if (elapsedTime>=2000)
//       {
//            // read servos temps
//            // read a first value, otherwise temp is false in second reading (0x2B)
//            arbotix->getDynamixelRegister(3,0x24,2);
//            fServo2Temp = servo2.getTemp();
//            fServo3Temp = servo3.getTemp();
//            //printf ("Temp Servo 2 = %i °C\n",fServo2Temp);
//            //printf ("Temp Servo 3 = %i °C\n",fServo3Temp);
//            ofResetElapsedTimeCounter() ;
//        }
    // set or get servos angles

    if (fbMotorsEnabled==false && arbotix->isInitialized())
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

    if (arbotix->isInitialized() && fbFirstArbotixConnection)
    {
        servo2.setPGain(32);
        fbFirstArbotixConnection = false;
    }
    //update servos
    if (arbotix->isInitialized() && fbMotorsEnabled) {
        //printf("set servo 2 angle to %f\n",fAngleServo2.get());
        servo1.setup(fMinServo1,fMaxServo1);
        servo2.setup(fMinServo2,fMaxServo2);
        servo3.setup(fMinServo3,fMaxServo3);
        servo4.setup(fMinServo4,fMaxServo4);
        servo5.setup(fMinServo5,fMaxServo5);

        if (fbLaunchBlockingMove==false)
        {
            servo1.setAngle(fAngleServo1.get());
            servo1.setSpeed(fSpeedServo1.get());

            servo2.setAngle(fAngleServo2.get());
            //int pG = servo2.getPGain();
            //ofLogNotice() << "PGain servo " << 2 << ":" << pG ;

            servo3.setAngle(fAngleServo3.get());
            //servo3.setPGain(32);
            //ofLogNotice() << "PGain servo " << 3 << ":" << pG ;

            servo4.setAngle(fAngleServo4.get());
            servo5.setAngle(fAngleServo5.get());

            // ---- launch non blocking move -------
            servo1.update();
            servo2.update();
            servo3.update();
            servo4.update();
            servo5.update();
            arbotix->moveServos();

        }
        else
        {
            //f3DCamera->stop();
            // ---- launch blocking move -------
            //ofResetElapsedTimeCounter() ;
            //printf("start move and wait\n");
            //boost::mutex::scoped_lock lock(fHeadTrackingMutex);
            bool success = servo1.moveAndWait(fAngleServo1.get());
            //int elapsedTime = ofGetElapsedTimeMillis();
            //ofLogNotice() << "move and wait 1 : "<< success << " - Elapsed time : " <<  elapsedTime ;


//            ofResetElapsedTimeCounter() ;
//            success = servo5.moveAndWait(fAngleServo5);
//            elapsedTime = ofGetElapsedTimeMillis();
//            ofLogNotice() << "move and wait 2 : "<< success << " - Elapsed time : " <<  elapsedTime ;

            //usleep(1000000);
            //f3DCamera->start();

            fbLaunchBlockingMove = false;
            fbTrackHead = true;
            //ofResetElapsedTimeCounter() ;
        }
     }

     //update Leds Values

     serialComArduino.writeByte('L');
     serialComArduino.writeByte((char) fRComponentLedValue.get());
     serialComArduino.writeByte((char)fVComponentLedValue.get());
     serialComArduino.writeByte((char)fBComponentLedValue.get());
     serialComArduino.writeByte('\n');

     serialComArduino.writeByte('B');
     serialComArduino.writeByte((char)fBrightnessLedValue.get());
     serialComArduino.writeByte('\n');

     if (fbExpressionEnabled == true)
     {
         serialComArduino.writeByte('E');
         //printf("send expression nb %i\n",fLedExpressionValue.get());
         serialComArduino.writeByte((char)fLedExpressionValue.get());
         serialComArduino.writeByte('\n');
     }
     else
     {
         serialComArduino.writeByte('N');
         serialComArduino.writeByte('\n');
     }

}

void ofApp::standUp()
{
//     fAngleServo3.set(1.0);
////     int timeSleepS = 1;
////     usleep(timeSleepS*1000000);
//     fAngleServo2.set(1.0);
//     fAngleServo4.set(0.8);
//     fAngleServo5.set(0.5);

    float currentVal1 = fAngleServo2.get();
    float currentVal2 = fAngleServo3.get();

    for (int i=0;i<1000;i++)
    {
        currentVal1 +=0.001;
        currentVal2 +=0.001;
        if (currentVal1>=1.0)
        {
            currentVal1 = 1.0;
        }
        if (currentVal2>=1.0)
        {
            currentVal2 = 1.0;
        }
    printf("set angle to %f\n",currentVal1);
    fAngleServo2.set(currentVal1);
    fAngleServo3.set(currentVal2);
    }

}

void ofApp::goToRest()
{
      fbTrackHead = false;

}



//--------------------------------------------------------------
void ofApp::draw(){
 _gui.draw();

    int elapsedTime = ofGetElapsedTimeMillis();

    // check servos temp

//    if (elapsedTime>=250)
//    {
//        servo2.getLoadInPct();
//        servo3.getLoadInPct();
//        ofResetElapsedTimeCounter() ;
//    }

    if (elapsedTime>=2000 && arbotix->isInitialized())
    {
            // read a first value, otherwise temp is false in second reading (0x2B)
            arbotix->getDynamixelRegister(3,0x24,2);

            int tempServo2 = servo2.getTemp();
            fServo2Temp.set(tempServo2);
            int tempServo3 = servo3.getTemp();
            fServo3Temp.set(tempServo3);
            //printf ("Temp Servo 2 = %f °C\n",fServo2Temp);
            //printf ("Temp Servo 3 = %f °C\n",fServo3Temp);
            ofResetElapsedTimeCounter() ;
    }

 // VISION
  if( cas == 2)
  {
     easyCam.begin();
     if (fbDrawCloud) {
         drawPointCloud();
         drawPoses();
     }
     easyCam.end();
     //drawReport();
  }
 
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

double ofApp::diffclock(clock_t clock1, clock_t clock2)
{
    double diffticks = clock1 - clock2;
    double diffms = (diffticks) / (CLOCKS_PER_SEC / 1000);
    return diffms;
}

void ofApp::enableMotors(bool &state)
{
    if (state==true)
    {
        fbMotorsEnabled = true;
    }
    else
    {
        servo3.disable();
        servo2.disable();
        servo4.disable();
        servo5.disable();
        servo1.disable();
        fbMotorsEnabled = false;
    }
}

void ofApp::enableDrawCloud(bool &state)
{
    if (state==true)
    {
        fbDrawCloud = true;
    }
    else
    {
        fbDrawCloud = false;
    }
}

void ofApp::enableHeadTracking(bool &state)
{
    //boost::mutex::scoped_lock lock(fHeadTrackingMutex);
    if (state==true)
    {
        fMinServo4.set(250);
        fMaxServo4.set(460);
        fMinServo5.set(420);
        fMaxServo5.set(580);
        fbTrackHead = true;

    }
    else
    {
        fMinServo4.set(fServosMins[3]);
        fMaxServo4.set(fServosMax[3]);
        fMinServo5.set(fServosMins[4]);
        fMaxServo5.set(fServosMax[4]);
        fbTrackHead = false;
    }
}

void ofApp::enableExpression(bool &state)
{
    if (state==true)
    {
        fbExpressionEnabled = true;
        printf("smile\n");

    }
    else
    {
        printf("not smile\n");
        fbExpressionEnabled = false;
    }

}

void ofApp::enableFindHead(bool &state)
{
    if (state==true)
    {
        fbFindHead = true;
    }
    else
    {
        fbFindHead = false;
    }
}

//void ofApp::enableMotors(bool state)
//{
//    if (state==true)
//    {
//        fbMotorsEnabled = true;
//    }
//    else
//    {
//        servo3.disable();
//        servo2.disable();
//        servo4.disable();
//        servo5.disable();
//        servo1.disable();
//        fbMotorsEnabled = false;
//    }
//}
//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    switch(key){

    case 'a':
        if (fbMotorsEnabled ==false)
        {
            bool val = true;
            enableMotors(val);

        }
        else if (fbMotorsEnabled==true)
        {
            bool val = false;
            enableMotors(val);
        }
        break;

    case 'b' :
    {
        printf("send Leds commands\n");
        serialComArduino.writeByte('L');
        serialComArduino.writeByte((char)ofRandom(0,255));
        serialComArduino.writeByte((char)ofRandom(0,255));
        serialComArduino.writeByte((char)ofRandom(0,255));
        serialComArduino.writeByte('\n');

   }

    case 'd' :
        if (fbDrawCloud ==false)
        {
            fbDrawCloud = true;
        }
        else if (fbDrawCloud==true)
        {
            fbDrawCloud = false;
        }
        break;


    case 'l' :
    {
        char val = 0x60;
        char picMess[picMsgLength];
        for (int i=0;i<picMsgLength;i++)
        {
            picMess[i]=val;
        }
        arbotix->sendMsgToPic(picId,picLedCmd,picMess);
        break;
    }

    case 'n' :
        {
        ofResetElapsedTimeCounter();
        arbotix->test();
        int elapsedTime = ofGetElapsedTimeMillis();
        printf("elapsed time = %i ms\n",elapsedTime);
        break;
        }


    case 's' :
        standUp();
        break;

    case 't' :
        if (fbTrackHead ==false)
        {
            fbTrackHead = true;
        }
        else if (fbTrackHead==true)
        {
            fbTrackHead = false;
        }
        break;

    case 1 :
        turnOnLed(1,'r',100);
        break;

    case 2 :
        turnOnLed(1,'v',100);
        break;

    case 3 :
        turnOnLed(1,'b',100);
        break;

    default:
        break;
    }


}

void ofApp::turnOnLed(const int &ledNb, const char &color, const int &intensity)
{
    char *picMsg = new char[picMsgLength];
    memset(picMsg,0,picMsgLength);

    switch (color){

    case 'r':
        picMsg[3*(ledNb-1)] = intensity;
        break;

    case 'v':
        picMsg[3*(ledNb-1)+1] = intensity;
        break;

    case 'b':
        picMsg[3*(ledNb-1)+2] = intensity;
        break;

    default :
        break;
    }

    arbotix->sendMsgToPic(picId,picLedCmd,picMsg);
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

              if (servoName!="" /*&& id!=0*/)
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
//--------------------------------------------------------------
void ofApp::calcAvgFPS() {
    int currMillis = ofGetElapsedTimeMillis();
    avgkFPS += (1000.0/(currMillis-lastMillis))/FPS_MEAN;
    lastMillis = currMillis;
    frameCount++;
    if (frameCount >= FPS_MEAN) {
        kFPS = avgkFPS;
        avgkFPS = frameCount =  0;
    }
}
//--------------------------------------------------------------
void ofApp::updateCloud() {
        //generate 3D image
        for(int y = 0; y < f3DImage.rows; y++)
        {
                Vec3f* Mi = f3DImage.ptr<Vec3f>(y);
                for(int x = 0; x < f3DImage.cols; x++){
                        ofVec3f thePoint = f3DCamera->getWorldCoordinateAt(x,y);

                        if ( (thePoint.z < g_max_z) && (thePoint.z > 0) ){
                                Mi[x][0] = thePoint.x;
                                Mi[x][1] = thePoint.y;
                                Mi[x][2] = thePoint.z;
                        }
                        else
                                Mi[x] = 0;
                }
        }
}
//--------------------------------------------------------------
void ofApp::drawPointCloud() {
        ofMesh mesh;

        mesh.setMode(OF_PRIMITIVE_POINTS);
        int step = 2;
        for(int y = 0; y < h; y += step) {
                for(int x = 0; x < w; x += step) {
                        if ((f3DCamera->getDistanceAt(x, y) > 0) && (f3DCamera->getDistanceAt(x, y) < g_max_z)) {
                                mesh.addColor(ofColor::grey);
//                                if (x==KW/2 && y==KH/2)
//                                {
//                                  printf("distance = %.1f\n",f3DCamera->getDistanceAt(KW/2,KH/2));
//                                }
                                mesh.addVertex(f3DCamera->getWorldCoordinateAt(x, y));
                        }
                }
        }
        ofPushMatrix();
        glPointSize(3);
        // the projected points are 'upside down' and 'backwards'
        ofScale(1, -1, -1);
        ofTranslate(0, 0, -1000); // center the points a bit
        glEnable(GL_DEPTH_TEST);
        mesh.drawVertices();
        glDisable(GL_DEPTH_TEST);
        ofPopMatrix();
}
//--------------------------------------------------------------
void ofApp::drawPoses() {
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofSetColor(0,0,255);
    glLineWidth(3);
    if(fHeadPoses.size()>0)
    {
            for(unsigned int i=0;i<1/*g_means.size()*/;++i)
            {
                ofVec3f pos = ofVec3f(fHeadPoses[i][0], fHeadPoses[i][1], fHeadPoses[i][2]);
                ofVec3f dir = ofVec3f(0,0,-150);
                dir.rotate(fHeadPoses[i][3], fHeadPoses[i][4], fHeadPoses[i][5]);
                dir += pos;
                ofLine(pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);

                double timeDiffMs = diffclock(clock(),objectDetectionStartTime);
                //printf("diff MS = %f\n",timeDiffMs);
                if (timeDiffMs>=50.0 /*and fbTrackHead==true*/)
                {
                    //printf("pos x = %.1f\n",pos.x);
                    //printf("pos y = %.1f\n",pos.y);
                    //printf("pos z = %.1f\n",pos.z);
                    //float xcm = (pos.x + float(w)/2)/37.795275590551;
                    //float ycm = (pos.y + float(h)/2)/37.795275590551;
                    ofPoint currentPos = pos;
                    float xm = (pos.x)/HFocalLength * pos.z;
                    float ym = (pos.y)/VFocalLength * pos.z;
                    fHeadHorizontalPos = atan(xm/pos.z) *180/PI; // pos in degrees
                    fHeadVerticalPos = -atan(ym/pos.z) *180/PI;

                    //printf("distance = %.1f\n",pos.z);
                    //printf("xm = %.1f\n",xm);
                    //printf("update head horizontal pos to %.1f\n",fHeadHorizontalPos);
                    //printf("update head vertical pos to %.1f\n",fHeadVerticalPos);

                    // display relative base angle
                    float servo1AngleDegrees = 512 + (fHeadHorizontalPos/180.0*3*512);
                    fHeadPositionX.set(servo1AngleDegrees);
                    //printf("angle vertical= %.1f\n",fHeadVerticalPos);

                    objectDetectionStartTime = clock();

                }
           }
        }
        ofPopMatrix();
}
//--------------------------------------------------------------
void ofApp::drawReport() {
    ofPushMatrix();
    ofSetColor(0);
    char reportStr[1024];
    sprintf(reportStr, "framecount: %i   FPS: %.2f", frameCount, kFPS);
    ofDrawBitmapString(reportStr, 10, 10);
    ofPopMatrix();
}
