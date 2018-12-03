#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "arbotixController.h"
#include "servo.h"
#include "ofxXmlSettings.h"

#include <boost/shared_ptr.hpp>
#include "ofParameterGroup.h"
#include "ofParameter.h"
#include "ofxOscParameterSync.h"

#include "ofxKinect.h"
#include "headposedetector.h"

//#include "ofxCv.h"
#include "ofxOpenCv.h"

#include "camera3d.h"

using namespace std;
using namespace cv;

const int kNbOfServos = 5;

// kinect frame width
#define KW 480
// kinect frame height
#define KH 360
//maximum distance form the sensor - used to segment the person
#define g_max_z 2000
// for the average fps calculation
#define FPS_MEAN 30


class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
                void exit();
                void standUp();
                void goToRest();

                void loadConfiguration(const std::string &fileName);
                //void enableMotors(bool state);
                void enableMotors(bool &state);
                void enableDrawCloud(bool &state);
                void enableFindHead(bool &state);
                void enableHeadTracking(bool &state);


                void turnOnLed(const int &pinNb);

                double diffclock(clock_t clock1, clock_t clock2);


                //Kinect head pose estimator
                void drawPointCloud();
                void drawPoses();
                void calcAvgFPS();
                void drawReport();
                void updateCloud();

                arbotixController *arbotix;
                ofArduino ard;
                ofxPanel _gui;

                //Control Params
                ofParameterGroup fGlobalControls;
                ofParameterGroup fAngleControl1;
                ofParameterGroup fMinMaxControl1;
                ofParameterGroup fAngleControl2;
                ofParameterGroup fMinMaxControl2;
                ofParameterGroup fAngleControl3;
                ofParameterGroup fMinMaxControl3;
                ofParameterGroup fAngleControl4;
                ofParameterGroup fMinMaxControl4;
                ofParameterGroup fAngleControl5;
                ofParameterGroup fMinMaxControl5;
                ofParameterGroup fBooleanControls;

                ofParameterGroup fHeadPositionControl;

                ofParameter <float> fAngleServo1;
                ofParameter <int> fMinServo1;
                ofParameter <int> fMaxServo1;
                ofParameter <float> fAngleServo2;
                ofParameter <int> fMinServo2;
                ofParameter <int> fMaxServo2;
                ofParameter <float> fAngleServo3;
                ofParameter <int> fMinServo3;
                ofParameter <int> fMaxServo3;
                ofParameter <float> fAngleServo4;
                ofParameter <int> fMinServo4;
                ofParameter <int> fMaxServo4;
                ofParameter <float> fAngleServo5;
                ofParameter <int> fMinServo5;
                ofParameter <int> fMaxServo5;

                ofParameter <int> fHeadPositionX;
                ofParameter <int> fHeadPositionY;

                ofParameter <bool> fTorqueControl;
                ofParameter <bool> fbMotorsEnabled;
                ofParameter <bool> fbDrawCloud;
                ofParameter <bool> fbFindHead;
                ofParameter <bool> fbTrackHead;

                ofxOscParameterSync iccoreConnexion;

                // Servos params
                ofxXmlSettings fXMLReader;
                std::vector<std::string> fServosNames;
                std::vector<int> fServosPins;
                std::vector<int> fServosIds;
                std::vector<int> fServosMins;
                std::vector<int> fServosMax;
                std::vector<int> fServosInitialPos;

                std::string fArbotixPortName;
                int fArbotixRate;

                std::string fArduinoPortName;
                int fArduinoRate;

                float fmeanHeadPositionX;
                float fmeanHeadPositionY;

                float fInitialPosServo1;
                float fInitialPosServo2;
                float fInitialPosServo3;
                float fInitialPosServo4;
                float fInitialPosServo5;



                //vector<boost::shared_ptr<servo> > fServosList;

                servo servo1;
                servo servo2;
                servo servo3;
                servo servo4;
                servo servo5;

                int fServo1Temp;
                int fServo2Temp;
                int fServo3Temp;
                int fServo4Temp;
                int fServo5Temp;

                std::vector <servo*> fServosList;

                ofTrueTypeFont	verdana14Font;

                std::string fRemoteControllerIP;
                int fRemoteControllerListeningPort;

                int w,h, cas;
                clock_t objectDetectionStartTime;


                // kinect
                camera3D *f3DCamera;
                ofEasyCam easyCam;
                //input 3D image
                Mat f3DImage;
                //Estimator parameters
                headPoseDetector *fHeadPoseDetector;
                std::vector< cv::Vec<float,POSE_SIZE>> fHeadPoses;

                //-------------------------------------------------------
                // kinect motor tilt angle
                int kTilt = 0;
                // stuff to calculate average fpg
                float kFPS = 0;
                float avgkFPS = 0;
                int frameCount = 0;
                int lastMillis = 0;
};
