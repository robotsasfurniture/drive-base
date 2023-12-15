#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

const int dacUnitsPerVolt = 280; // increasing this number decreases the calculated voltage
const byte batMonitorPin = 35;

const int encoderTicksPerRev = 530;
JTwoDTransform robotToWheelScalar = { 1, 1, 1 }; // adjust until it converts robot speed in your chosen units to wheel units (increasing numbers makes robot faster)

#define blEncPins 36, 39
#define flEncPins 19, 27
#define frEncPins 22, 34
#define brEncPins 23, 21 // one pin bad

// byte _pinPosCh, byte _pinPos, byte _pinNeg, byte _pinNegCh
#define blMotPins 8, 25, 26, 9
#define flMotPins 10, 33, 32, 11
#define frMotPins 12, 5, 18, 13
#define brMotPins 14, 16, 17, 15

JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);

JMotorDriverEsp32HBridge blMotorDriver = JMotorDriverEsp32HBridge(blMotPins, false, 2000, 12, false);
JMotorDriverEsp32HBridge flMotorDriver = JMotorDriverEsp32HBridge(flMotPins, false, 2000, 12, false);
JMotorDriverEsp32HBridge frMotorDriver = JMotorDriverEsp32HBridge(frMotPins, false, 2000, 12, false);
JMotorDriverEsp32HBridge brMotorDriver = JMotorDriverEsp32HBridge(brMotPins, false, 2000, 12, false);

JEncoderQuadratureAttachInterrupt blEncoder = JEncoderQuadratureAttachInterrupt(blEncPins, 1.0 / encoderTicksPerRev, true, 200000);
JEncoderQuadratureAttachInterrupt flEncoder = JEncoderQuadratureAttachInterrupt(flEncPins, 1.0 / encoderTicksPerRev, true, 200000);
JEncoderQuadratureAttachInterrupt frEncoder = JEncoderQuadratureAttachInterrupt(frEncPins, 1.0 / encoderTicksPerRev, true, 200000);
JEncoderQuadratureAttachInterrupt brEncoder = JEncoderQuadratureAttachInterrupt(brEncPins, 1.0 / encoderTicksPerRev, true, 200000);

JMotorCompBasic motorCompensator = JMotorCompBasic(voltageComp, 3, 0.5); // volts per rps, min rps

JControlLoopBasic flCtrlLoop = JControlLoopBasic(30);
JControlLoopBasic frCtrlLoop = JControlLoopBasic(30);
JControlLoopBasic blCtrlLoop = JControlLoopBasic(30);
JControlLoopBasic brCtrlLoop = JControlLoopBasic(30);

JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, motorCompensator, flEncoder, flCtrlLoop, INFINITY, INFINITY, 0.15);
JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, motorCompensator, frEncoder, frCtrlLoop, INFINITY, INFINITY, 0.15);
JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, motorCompensator, blEncoder, blCtrlLoop, INFINITY, INFINITY, 0.15);
JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, motorCompensator, brEncoder, brCtrlLoop, INFINITY, INFINITY, 0.15);

JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrain, { INFINITY, INFINITY, INFINITY }, { INFINITY, INFINITY, INFINITY }, { INFINITY, INFINITY, INFINITY }, false);

boolean enabled = false;
boolean wasEnabled = false;
const byte ONBOARD_LED = 2;

byte mode = 0;

JTwoDTransform driverInputWifi = JTwoDTransform();

void Enabled()
{
    // code to run while enabled, put your main code here
    if (mode == 1) { // drive
        drivetrainController.moveVel(JDeadzoneRemover::calculate(driverInputWifi, { 0, 0, 0 }, drivetrainController.getMaxVel() * .85, { .02, .02, .02 }));
    } else {
        drivetrainController.moveVel({ 0, 0, 0 });
    }
}

void Enable()
{
    // turn on outputs
    drivetrainController.resetDist();
    drivetrainController.moveDist({ 0, 0, 0 });
    drivetrainController.enable();
    driverInputWifi = { 0, 0, 0 };
}

void Disable()
{
    // shut off all outputs
    drivetrainController.disable();
    driverInputWifi = { 0, 0, 0 };
}

jENCODER_MAKE_ISRS_MACRO(blEncoder);
jENCODER_MAKE_ISRS_MACRO(flEncoder);
jENCODER_MAKE_ISRS_MACRO(frEncoder);
jENCODER_MAKE_ISRS_MACRO(brEncoder);
void PowerOn()
{
    blMotorDriver.pwmDriverPos.disableState = HIGH;
    blMotorDriver.pwmDriverNeg.disableState = HIGH;
    flMotorDriver.pwmDriverPos.disableState = HIGH;
    flMotorDriver.pwmDriverNeg.disableState = HIGH;
    frMotorDriver.pwmDriverPos.disableState = HIGH;
    frMotorDriver.pwmDriverNeg.disableState = HIGH;
    brMotorDriver.pwmDriverPos.disableState = HIGH;
    brMotorDriver.pwmDriverNeg.disableState = HIGH;

    // runs once on robot startup, set pin modes and use begin() if applicable here
    blEncoder.setUpInterrupts(blEncoder_jENCODER_ISR_A, blEncoder_jENCODER_ISR_B);
    flEncoder.setUpInterrupts(flEncoder_jENCODER_ISR_A, flEncoder_jENCODER_ISR_B);
    frEncoder.setUpInterrupts(frEncoder_jENCODER_ISR_A, frEncoder_jENCODER_ISR_B);
    brEncoder.setUpInterrupts(brEncoder_jENCODER_ISR_A, brEncoder_jENCODER_ISR_B);
}

void Always()
{
    drivetrainController.run();
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
    EWD::sendFl(drivetrainController.getVel().x);
    EWD::sendFl(drivetrainController.getVel().y);
    EWD::sendFl(drivetrainController.getVel().theta);
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
