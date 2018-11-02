#ifndef SERVO_H
#define SERVO_H


#include "ofMain.h"
#include "arbotixController.h"

class servo
{
public:
    servo();
    void update();
    void draw();
    void setController(arbotixController *controller) {fController = controller;};
    void setup(int min, int max);
    void setAngle(float angle);
    void setId(const int &id);
    void setSpeed(const int &speed);
    void setName(const std::string &name);
    int  getId();
    int getTemp();
    int getPos();
    void enable();
    void disable();
    std::string getName();

private:
    arbotixController *fController;
    int fAngle;
    int  fMin;
    int  fMax;
    int fId;
    std::string fName;
    int fSpeed;
    bool fIsInitialized;
    bool fIsEnabled;

};

#endif // SERVO_H
