#ifndef CAMERA3D_H
#define CAMERA3D_H

#include "ofxKinect.h"

class camera3D
{
public:
    camera3D();
    camera3D(const std::string &Ip, const int &listeningPort, const int &width, const int &height);
    void setup();
    void start();
    void stop();
    void update();
    void draw(int x, int y);
    float   getDistanceAt(int x, int y);
    float   getDistanceAt(const ofPoint &p);
    ofVec3f getWorldCoordinateAt(int cx, int cy);
    bool    setCameraTiltAngle(float angleInDegrees); // to do
    bool isFrameNew();

private :
    ofxKinect* fLocalCam;
    bool fbLocalCamera;
};

#endif // CAMERA3D_H
