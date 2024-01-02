#include <iostream>
#include "MotorModel.hh"

int main()
{
    double torqueConstant = 1;
    double momentOfInertia = 2;
    double viscousFrictionCoefficient = 3;
    MotorModel motorModel(torqueConstant, momentOfInertia, viscousFrictionCoefficient);

    printf("時刻,入力電流[A],位置[rad],角速度[rad/s],角加速度[rad/s²]\n");

    for (int msec = 0; msec <= 10000; msec += 100)
    {
        double currentTime = msec * 1.0e-3;
        double inputCurrent = 1000 <= msec && msec <= 2000 ? 1.0 : 0;
        motorModel.calculatePosition(currentTime, inputCurrent);

        double position = motorModel.getLastPosition();
        double angularVelocity = motorModel.getLastAngularVelocity();
        double angularAcceleration = motorModel.getLastAngularAcceleration();

        printf(
            "%f,%f,%f,%f,%f\n",
            currentTime,
            inputCurrent,
            position,
            angularVelocity,
            angularAcceleration);
    }

    return 0;
}
