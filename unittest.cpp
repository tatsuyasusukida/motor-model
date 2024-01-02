#include <cassert>
#include <cstdio>
#include "MotorModel.hh"

bool g_debug = false;

#define ASSERT(actual, expected)                                         \
    if (g_debug)                                                         \
        printf("DEBUG: actual = %f, expected = %f\n", actual, expected); \
    assert(actual == expected);

#define TEST(fn)         \
    printf("%s\n", #fn); \
    fn();

void testCalculateDefiniteIntegralValue()
{
    double torqueConstant = 1;
    double momentOfInertia = 2;
    double viscousFrictionCoefficient = 3;
    MotorModel motorModel(torqueConstant, momentOfInertia, viscousFrictionCoefficient);

    double previousTime = 1;
    double previousValue = 2;
    double currentTime = 3;
    double currentValue = 4;
    double actual = motorModel.calculateDefiniteIntegralValue(
        previousTime,
        previousValue,
        currentTime,
        currentValue);

    double expected = (previousValue + currentValue) * (currentTime - previousTime) / 2;

    ASSERT(actual, expected)
}

void testCalculateTorque()
{
    double torqueConstant = 1;
    double momentOfInertia = 2;
    double viscousFrictionCoefficient = 3;
    MotorModel motorModel(torqueConstant, momentOfInertia, viscousFrictionCoefficient);

    double previousTime = 4;
    motorModel.setPreviousTime(previousTime);

    double currentTime = 5;
    double inputCurrent = 6;
    double actual = motorModel.calculateTorque(currentTime, inputCurrent);
    double expected = torqueConstant * inputCurrent;

    ASSERT(actual, expected)
}

void testCalculateAngularAcceleration()
{
    double torqueConstant = 1;
    double momentOfInertia = 2;
    double viscousFrictionCoefficient = 3;
    MotorModel motorModel(torqueConstant, momentOfInertia, viscousFrictionCoefficient);

    double previousTime = 4;
    motorModel.setPreviousTime(previousTime);

    double currentTime = 5;
    double inputCurrent = 6;
    double actual = motorModel.calculateAngularAcceleration(currentTime, inputCurrent);

    double torque = motorModel.getLastTorque();
    double frictionalTorque = motorModel.getViscousFrictionCoefficient() * motorModel.getLastAngularVelocity();
    double loadTorque = motorModel.getLoadTorque(currentTime);
    double expected = (torque - (frictionalTorque + loadTorque)) / motorModel.getMomentOfInertia();

    ASSERT(actual, expected)
}

void testCalculateAngularVelocity()
{
    double torqueConstant = 1;
    double momentOfInertia = 2;
    double viscousFrictionCoefficient = 3;
    MotorModel motorModel(torqueConstant, momentOfInertia, viscousFrictionCoefficient);

    double previousTime = 4;
    motorModel.setPreviousTime(previousTime);

    double currentTime = 5;
    double inputCurrent = 6;
    double previousAngularAcceleration = motorModel.getLastAngularAcceleration();
    double actual = motorModel.calculateAngularVelocity(currentTime, inputCurrent);

    double currentAngularAcceleration = motorModel.getLastAngularAcceleration();
    double expected = motorModel.calculateDefiniteIntegralValue(
        previousTime,
        previousAngularAcceleration,
        currentTime,
        currentAngularAcceleration);

    ASSERT(actual, expected)
}

void testCalculatePosition()
{
    double torqueConstant = 1;
    double momentOfInertia = 2;
    double viscousFrictionCoefficient = 3;
    MotorModel motorModel(torqueConstant, momentOfInertia, viscousFrictionCoefficient);

    double previousTime = 4;
    motorModel.setPreviousTime(previousTime);

    double currentTime = 5;
    double inputCurrent = 6;
    double previousAngularVelocity = motorModel.getLastAngularVelocity();
    double actual = motorModel.calculatePosition(currentTime, inputCurrent);

    double currentAngularVelocity = motorModel.getLastAngularVelocity();
    double expected = motorModel.calculateDefiniteIntegralValue(
        previousTime,
        previousAngularVelocity,
        currentTime,
        currentAngularVelocity);

    ASSERT(actual, expected)
}

int main()
{
    TEST(testCalculateDefiniteIntegralValue);
    TEST(testCalculateTorque);
    TEST(testCalculateAngularAcceleration);
    TEST(testCalculateAngularVelocity);
    TEST(testCalculatePosition);

    return 0;
}