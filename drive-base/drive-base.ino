#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

// const int dacUnitsPerVolt = 413; // increasing this number decreases the calculated voltage
const int encoderTicksPerRev = 530;
JTwoDTransform robotToWheelScalar = { 1, 1, 1 }; // adjust until it converts robot speed in your chosen units to wheel units (increasing numbers makes robot faster)

#define flEncPins 36, 39

#define flMotPins 0, 32, 33, 8

// JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
JVoltageCompConst voltageComp = JVoltageCompConst(12);
JMotorDriverEsp32HBridge flMotorDriver = JMotorDriverEsp32HBridge(flMotPins);
// JMotorDriverEsp32L293 frMotorDriver = JMotorDriverEsp32L293(portB, true, false, false, 8000, 12);
// JMotorDriverEsp32L293 blMotorDriver = JMotorDriverEsp32L293(portC, true, false, false, 8000, 12);
// JMotorDriverEsp32L293 brMotorDriver = JMotorDriverEsp32L293(portD, true, false, false, 8000, 12);
JEncoderQuadratureAttachInterrupt flEncoder = JEncoderQuadratureAttachInterrupt(flEncPins, 1.0 / encoderTicksPerRev, true, 20000);
// JEncoderSingleAttachInterrupt frEncoder = JEncoderSingleAttachInterrupt(inport2, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
// JEncoderSingleAttachInterrupt blEncoder = JEncoderSingleAttachInterrupt(inport3, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
// JEncoderSingleAttachInterrupt brEncoder = JEncoderSingleAttachInterrupt(port3Pin, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
// JMotorCompBasic motorCompensator = JMotorCompBasic(voltageComp, 1.7, 0.5); // volts per rps, min rps
// JControlLoopBasic flCtrlLoop = JControlLoopBasic(10, 400);
// JControlLoopBasic frCtrlLoop = JControlLoopBasic(10, 400);
// JControlLoopBasic blCtrlLoop = JControlLoopBasic(10, 400);
// JControlLoopBasic brCtrlLoop = JControlLoopBasic(10, 400);
// JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, motorCompensator, flEncoder, flCtrlLoop, INFINITY, INFINITY, 0.25;
// JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, motorCompensator, frEncoder, frCtrlLoop, INFINITY, INFINITY, 0.25);
// JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, motorCompensator, blEncoder, blCtrlLoop, INFINITY, INFINITY, 0.25);
// JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, motorCompensator, brEncoder, brCtrlLoop, INFINITY, INFINITY, 0.25);
// JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
// JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrain, { INFINITY, INFINITY, INFINITY }, { 5, 5, 5 }, { 0.15, 0.15, 0.25 }, false);

boolean enabled = false;
boolean wasEnabled = false;
const byte ONBOARD_LED = 2;

byte mode = 0;

JTwoDTransform driverInputWifi = JTwoDTransform();

void Enabled()
{
    // code to run while enabled, put your main code here
    // if (mode == 1) { // drive
    //     drivetrainController.moveVel(JDeadzoneRemover::calculate(driverInputWifi, { 0, 0, 0 }, drivetrainController.getMaxVel(), { .02, .02, .02 }));
    // } else {
    //     drivetrainController.moveVel({ 0, 0, 0 });
    // }
}

void Enable()
{
    // turn on outputs
    // drivetrainController.resetDist();
    // drivetrainController.moveDist({ 0, 0, 0 });
    // drivetrainController.enable();
    driverInputWifi = { 0, 0, 0 };
    flMotorDriver.enable();
}

void Disable()
{
    // shut off all outputs
    // drivetrainController.disable();
    driverInputWifi = { 0, 0, 0 };
    flMotorDriver.disable();
}

jENCODER_MAKE_ISRS_MACRO(flEncoder);
// jENCODER_MAKE_ISRS_MACRO(frEncoder);
// jENCODER_MAKE_ISRS_MACRO(brEncoder);
// jENCODER_MAKE_ISRS_MACRO(blEncoder);
void PowerOn()
{
    flMotorDriver.pwmDriverPos.disableState = HIGH;
    flMotorDriver.pwmDriverNeg.disableState = HIGH;

    // runs once on robot startup, set pin modes and use begin() if applicable here
    flEncoder.setUpInterrupts(flEncoder_jENCODER_ISR_A, flEncoder_jENCODER_ISR_B);
    // frEncoder.setUpInterrupts(frEncoder_jENCODER_ISR_A, frEncoder_jENCODER_ISR_B);
    // brEncoder.setUpInterrupts(brEncoder_jENCODER_ISR, brEncoder_jENCODER_ISR_B);
    // blEncoder.setUpInterrupts(blEncoder_jENCODER_ISR, blEncoder_jENCODER_ISR_B);
}

void Always()
{
    // drivetrainController.run();
    flMotorDriver.set(driverInputWifi.y);
    delay(1);
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;
    EWD::signalLossTimeout = 350;
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    mode = EWD::recvBy();
    driverInputWifi.x = EWD::recvFl();
    driverInputWifi.y = EWD::recvFl();
    driverInputWifi.theta = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendFl(flEncoder.getPos());
    EWD::sendFl(flEncoder.getVel());
    EWD::sendFl(driverInputWifi.y);
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
