/**
DRIVE BASE - ROBOTS AS FURNITURE
by joshua-8 (joshuaphelps127@gmail.com) fall 2023 for Professor Ian Gonsher's Robots as Furniture project at Brown University
this program controls a mecanum drive base for the robots as furniture project
this program is based on the RCMv2 template https://github.com/rcmgames/rcmv2
this program can be controlled with the https://github.comrcmgames/rcmds driver station
*/
#define ONBOARD_LED 13
#define batMonitorPin 36

boolean enabled = false;
boolean wasEnabled = false;

#include <ESP32_easy_wifi_data.h> //communication with driver station //https://github.com/joshua-8/ESP32_easy_wifi_data v1.1.1
#include <JMotor.h> //https://github.com/joshua-8/JMotor //controlling motors //TODO: version
#include <ros.h> //communication with raspberry pi //https://github.com/frankjoshua/rosserial_arduino_lib v0.9.1

const int dacUnitsPerVolt = 380; // increasing this number decreases the calculated voltage

const int encoderTicksPerRev = 2340;

JTwoDTransform robotToWheelScalar = { 1, 1, 1 }; // adjust until it converts robot speed in your chosen units to wheel rotations (increasing numbers makes robot faster)

JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt); // measures the battery voltage
JMotorDriverEsp32L293 frMotorDriver = JMotorDriverEsp32L293(1, 1, 1, 1, true, false, false, 8000, 12); // controls the motor drivers
JMotorDriverEsp32L293 flMotorDriver = JMotorDriverEsp32L293(2, 2, 2, 2, true, false, false, 8000, 12);
JMotorDriverEsp32L293 blMotorDriver = JMotorDriverEsp32L293(3, 3, 3, 3, true, false, false, 8000, 12);
JMotorDriverEsp32L293 brMotorDriver = JMotorDriverEsp32L293(4, 4, 4, 4, true, false, false, 8000, 12);
JEncoderSingleAttachInterrupt frEncoder = JEncoderSingleAttachInterrupt(5, (float)1 / encoderTicksPerRev, false, 200000, 25, FALLING); // reads the encoders
JEncoderSingleAttachInterrupt flEncoder = JEncoderSingleAttachInterrupt(6, (float)1 / encoderTicksPerRev, false, 200000, 25, FALLING);
JEncoderSingleAttachInterrupt blEncoder = JEncoderSingleAttachInterrupt(7, (float)1 / encoderTicksPerRev, false, 200000, 25, FALLING);
JEncoderSingleAttachInterrupt brEncoder = JEncoderSingleAttachInterrupt(8, (float)1 / encoderTicksPerRev, false, 200000, 25, FALLING);
JMotorCompStandardConfig frMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100); // converts from motor speed to voltage
JMotorCompStandardConfig flMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandardConfig blMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandardConfig brMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandard frMotorCompensator = JMotorCompStandard(voltageComp, frMotorConfig); // converts from voltage to motor driver input: fraction of battery voltage
JMotorCompStandard flMotorCompensator = JMotorCompStandard(voltageComp, flMotorConfig);
JMotorCompStandard blMotorCompensator = JMotorCompStandard(voltageComp, blMotorConfig);
JMotorCompStandard brMotorCompensator = JMotorCompStandard(voltageComp, brMotorConfig);
JControlLoopBasic frCtrlLoop = JControlLoopBasic(7); // proportional control loop
JControlLoopBasic flCtrlLoop = JControlLoopBasic(7);
JControlLoopBasic blCtrlLoop = JControlLoopBasic(7);
JControlLoopBasic brCtrlLoop = JControlLoopBasic(7);
JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, frMotorCompensator, frEncoder, frCtrlLoop); // motor controller class combines everything
JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, flMotorCompensator, flEncoder, flCtrlLoop);
JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, blMotorCompensator, blEncoder, blCtrlLoop);
JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, brMotorCompensator, brEncoder, brCtrlLoop);
JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrain, { 1, 1, 1 }, { 1, 1, 1 }, { 1, 1, 1 }); // provides acceleration limiting

void Enabled()
{
    // code to run while enabled, put your main code here
}

void Enable()
{
    // turn on outputs
    drivetrainController.enable();
}

void Disable()
{
    // shut off all outputs
    drivetrainController.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
}

void Always()
{
    // always runs if void loop is running, top level JMotor run() functions should be put here
    drivetrainController.run();

    delay(1);
}

void configWifi()
{
    // EWD::mode = EWD::Mode::connectToNetwork;
    // EWD::routerName = "router";
    // EWD::routerPassword = "password";
    // EWD::routerPort = 25210;

    EWD::mode = EWD::Mode::createAP;
    EWD::APName = "table";
    EWD::APPassword = "Gonsher";
    EWD::APPort = 25210;
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
}

////////////////////////////// you don't need to edit below this line ////////////////////

void setup()
{
    Serial.begin(115200);
    pinMode(ONBOARD_LED, OUTPUT);
    PowerOn();
    Disable();
    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

void loop()
{
    EWD::runWifiCommunication();
    if (!EWD::wifiConnected || EWD::timedOut()) {
        enabled = false;
    }
    Always();
    if (enabled && !wasEnabled) {
        Enable();
    }
    if (!enabled && wasEnabled) {
        Disable();
    }
    if (enabled) {
        Enabled();
        digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
    } else {
        if (!EWD::wifiConnected)
            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
        else if (EWD::timedOut())
            digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
        else
            digitalWrite(ONBOARD_LED, HIGH); // on, disabled
    }
    wasEnabled = enabled;
}
