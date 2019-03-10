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

int servo::getLoad()
{
    int load = fController->getServoLoad(fId);
    //printf("servo %i - load = %i\n",fId,load);
    return load;
}

float servo::getLoadInPct()
{
    int load = fController->getServoLoad(fId);
    float pctLoad = load/1023.0*100.0;
    //printf("servo %i - load = %1.1f\n",fId,pctLoad);
    return pctLoad;
}

int servo::getPos()
{
    int pos = fController->getServoPos(fId);
    return pos;
}

int servo::getPGain()
{
    int pG = fController->getServoPGain(fId);
    return pG;
}


void servo::setAngle(float angle)
{
    fAngle = (int) ofMap(angle,0.,1.,fMin,fMax);
//    if (fId==1)
//    {
//        printf("servo::setAngle - fMin = %i - fMax = %i - angle = %i\n",fMin,fMax,fAngle);
//    }
}

void servo::waitForStop()
{
    if (fIsInitialized)
    {
        if (fController==NULL)
        {
         ofLogError("Servo Controller not set");
        }
        else
        {
            fController->waitForDynamixelStopped((unsigned char)fId);
        }
    }
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

bool servo::moveAndWait(const float & angle)
{
    //printf ("servo::moveAndWait - angle = %f, ",angle);
    this->setAngle(angle);
    //printf ("move servo to %i - and wait, ",fAngle);
    int ret = fController->moveServoAndWait(fId,fAngle,fSpeed);
    return ret;
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


void servo::setPGain(const int &pGain)
{
    fController->setPGain(fId,pGain);
}


int servo::getId()
{
    ofLogNotice()  << "servo::getId() - return : " << fId;
    return fId;
}

void servo::draw()
{

}

