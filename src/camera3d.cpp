#include "camera3d.h"


camera3D::camera3D(const std::string &Ip,
                   const int &listeningPort,
                   const int &width,
                   const int &height)
{

     //fRemoteCam = new ofxKinectClient(Ip, listeningPort, width, height);
     //fbLocalCamera = False;

     printf("CAM IS REMOTE\n");
}

camera3D::camera3D()
{
    fLocalCam = new ofxKinect();
    fbLocalCamera = true;
    printf("CAM IS LOCAL\n");
}

void camera3D::setup()
{

}

void camera3D::start()
{
    if (fbLocalCamera)
    {
        //fLocalCam->setRegistration(true);
        fLocalCam->init();
        fLocalCam->open();
        fLocalCam->setDepthClipping(500,2000);
        printf("kinext width = %i\n",fLocalCam->width);
        printf("kinext height = %i\n",fLocalCam->height);

    }
}

void camera3D::stop()
{
    if (!fbLocalCamera)
    {
        //fRemoteCam->stop();
    }
}

void camera3D::update()
{
    if (fbLocalCamera)
    {
        fLocalCam->update();
    }
}

void camera3D::draw(int x, int y)
{
    if (fbLocalCamera)
    {
        fLocalCam->draw(x,y);
    }

}

float camera3D::getDistanceAt(int x, int y)
{
    if (fbLocalCamera)
    {
        return fLocalCam->getDistanceAt(x,y);
    }
    else
    {
        return 0.0;
    }
}

float camera3D::getDistanceAt(const ofPoint &p)
{
    if (fbLocalCamera)
    {
        return fLocalCam->getDistanceAt(p);
    }
    else
    {
        return 0.0;
    }
}


ofVec3f camera3D::getWorldCoordinateAt(int x, int y)
{
    if (fbLocalCamera)
    {
        return fLocalCam->getWorldCoordinateAt(x,y);
    }
    else
    {
        return ofVec3f(0.0,0.0,0.0);
    }
}

bool camera3D::isFrameNew()
{
    if (fbLocalCamera)
    {
        return fLocalCam->isFrameNew();
    }
    else
    {
        return false;
    }
}
