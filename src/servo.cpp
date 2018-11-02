#include "servo.h"

servo::servo():
fIsInitialized(false),
fIsEnabled(false)
{

}

void servo::enable()
{
    if (!fIsEnabled)
    {
        fController->enableServo(fId);
        fIsEnabled = true;
    }
}

void servo::disable()
{
//    if (fIsEnabled)
//    {
        fController->disableServo(fId);
        fIsEnabled = false;
//    }
}

void servo::setup(int min, int max)
{
    fMin=min;
    fMax=max;
    fIsInitialized = true;
}

int servo::getTemp()
{
    int temp = fController->getServoTemp(fId);
    return temp;
}

int servo::getPos()
{
    int pos = fController->getServoPos(fId);
    return pos;
}

void servo::setAngle(float angle)
{
    //printf("servo::setAngle - fMin = %i - fMax = %i - angle = %f\n",fMin,fMax,angle);
    fAngle = (int) ofMap(angle,0.,1.,fMin,fMax);
}


void servo::update()
{

    if (fIsInitialized)
    {
        //printf("servo angle :%i\n",fAngle);
        int angle = fAngle ; //(int)ofMap(fAngle, 0., 1., fMin, fMax);
        if (fController==NULL)
        {
         ofLogError("Servo Controller not set");
        }
        else
        {
            //printf("min servo : %i\n",fMin);
            //printf("max servo : %i\n",fMax);
            //printf("send servo angle : %i\n",angle);
            fController->sendServoAngle(fId, angle, fSpeed);
        }
    }
}

void servo::setName(const std::string &name)
{
    fName = name;
}

std::string servo::getName()
{

    return fName;
}

void servo::setId(const int &id)
{
    ofLogNotice()  << "servo::setId() to : " << id;
    fId = id;
}


void servo::setSpeed(const int &speed)
{
    fSpeed = speed;
}

int servo::getId()
{
    ofLogNotice()  << "servo::getId() - return : " << fId;
    return fId;
}

void servo::draw()
{

}

