#include <avr/sleep.h>
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LCD_BACK_LIGHT_SENS_DEGREE (0.15) //sensevity of when to turn on backlight
#define SENSOR_6050_FIFO_HZ 20 //my update to motion_api, should be defined prior include
#define USE_LCD
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define READINGS_AMOUNT_AVR static_cast<uint8_t>((SENSOR_6050_FIFO_HZ) / 10 + 3) //how many reading to use to calc avr
//#define READINGS_AMOUNT_AVR 3

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#include "elapsedMillis/elapsedMillis.h"
#include "my_owns/circular.h"
#include "simple_comm.h"

#ifdef USE_LCD
#include "LiquidCrystal.h"
LiquidCrystal lcd(4, 5, 10, 11, 12, 13);
#define MAX_BRIGHTNESS 50
#define backLightPin 3
#endif

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t  devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)



inline void PersonalSensorCalibrate()
{
    // supply your own gyro offsets here, scaled for min sensitivity
    //to get those sensor must be aligned and other scetch launched like 6050 example_raw
    //or http://wired.chillibasket.com/2015/01/calibrating-mpu6050/
    //those magic numbers are PERSONAL for each new device/board
    mpu.setXGyroOffset(-63);
    mpu.setYGyroOffset(-41);
    mpu.setZGyroOffset(8);

    mpu.setXAccelOffset(418);
    mpu.setYAccelOffset(-622);
    mpu.setZAccelOffset(1173);
}

void printValues(float azimuth, float excl, float t = 0)
{
#ifdef USE_LCD
    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print(F("Azim: "));
//    lcd.print(azimuth);
//    lcd.setCursor(0, 1);
//    lcd.print(F("Excl: "));
//    lcd.print(excl);
    lcd.setCursor(0, 0);
    lcd.print(F("A/E:"));
    lcd.print(degrees(azimuth), 1);
    //lcd.print(static_cast<int>(azimuth));
    lcd.print(F("/"));
    lcd.print(degrees(excl), 1);
    //lcd.print(static_cast<int>(excl));

    lcd.setCursor(0, 1);
    lcd.print(F("t: "));
    lcd.print(t, 1);
    lcd.print(F(" C"));
#endif
};

void printError(const String& text, int code)
{
#ifdef USE_LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(text);
    lcd.setCursor(0, 1);
    lcd.print(F("Code: "));
    lcd.print(code);
#endif
}

void dumpXYZ(const VectorFloat& arr)
{
#ifdef USE_LCD
    lcd.setCursor(0, 1);
    lcd.print(arr.x);lcd.print(F(" "));lcd.print(arr.y);lcd.print(F(" "));lcd.print(arr.z);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile uint8_t sensorDataReady = 0;
void InterruptDataReady()
{
    sensorDataReady = 1;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
#ifdef USE_LCD
    pinMode(backLightPin, OUTPUT);
    analogWrite(backLightPin, MAX_BRIGHTNESS);
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print(F("Init,fifo: "));
    lcd.print(dmpConfig[MPU6050_DMP_CONFIG_SIZE - 1]);

#endif
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    mpu.reset();
    delay(100);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    PersonalSensorCalibrate();

    mpu.setTempSensorEnabled(true);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        // enable Arduino interrupt detection

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), InterruptDataReady, RISING);
        // get expected DMP packet size for later comparison
#ifdef USE_LCD
        lcd.setCursor(0, 1);
        lcd.print(F("Calibrating..."));
#endif
    }
    else
    {
        printError(F("DMP init failed"), devStatus);
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
    }
}

static float az0 = 0;
static float el0 = 0;
static ard_st::Message msg;

float getAz(float az1)
{
    while (az1 < 0.f) az1 += M_PI * 2.f;
    return my_helpers::removeRotRad<decltype(az0)>(az1);
}


void readSensor(my_helpers::Circular<decltype(az0), READINGS_AMOUNT_AVR>& az, my_helpers::Circular<decltype(el0), READINGS_AMOUNT_AVR>& el)
{
    using namespace my_helpers;
    sensorDataReady = false;
    uint8_t mpuIntStatus = mpu.getIntStatus();
    int16_t fifoCount    = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printError(F("FIFO overflow!"), -1);
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else
    {
        if (mpuIntStatus & 0x02)
        {
            // wait for correct available data length, should be a VERY short wait
            do
            {
                static uint8_t  fifoBuffer[64]; // FIFO storage buffer
                static Quaternion q;           // [w, x, y, z]         quaternion container
                static VectorFloat gravity;

                if (fifoCount < packetSize)
                    break;

                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
                mpu.dmpGetQuaternion(msg.message.value.vals.current_quat, fifoBuffer);
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                q.normalize();

                mpu.dmpGetGravity(&gravity, &q);
                gravity.normalize();

                //https://www.reddit.com/r/Astronomy/comments/3udenf/quaternion_matrix_or_euler_angles_conversion_to/
                auto elt = removeRotRad<decltype(el0)>(atan(gravity.x / sqrt(sqrf(gravity.y) + sqrf(gravity.z))));
                el.push_back(elt);

                decltype(az0) azt;

                // if (elt > radians(45))
                azt = -2.f * atan2(q.z, q.w);
                // else
                // azt = atan2(2 * (q.x* q.y -  q.w * q.z), 2 * (q.w * q.w +  q.x * q.x) - 1);
                az.push_back(getAz(azt));
            }
            while (fifoCount > 0);
        }
    }
    Serial.begin(115200);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop()
{
    using az_t = decltype(az0);

    static bool once = true;
    static elapsedMillis timeElapsed;
    static elapsedMillis light;
    static elapsedMillis hadUsb;
    static uint32_t counter = 0;

    static my_helpers::Circular<az_t , READINGS_AMOUNT_AVR> az(0);
    static my_helpers::Circular<az_t , READINGS_AMOUNT_AVR> el(0);

    const static float lightSens = radians(LCD_BACK_LIGHT_SENS_DEGREE);
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    while (sensorDataReady)
        readSensor(az, el);


    if (once)
    {
        //let it stabilize for 5seconds
        if (timeElapsed > 10000 + 1000 * (200 / SENSOR_6050_FIFO_HZ ))
        {
            //after pressing "reset" current position will be used as zero
            az0 = az;
            el0 = el;
            az.clear(az0);
            el.clear(el0);
            timeElapsed = 0;
            light = 0;
            hadUsb = 60000;
            interrupts();
            once = false;
        }
    }
    else
    {

        interrupts();
        while (Serial.available() >= 3 + 9)
        {
            interrupts();
            if (Serial.find(MESSAGE_HDR))
            {
                //computer makes S/R request, and arduino just responds
                Serial.readBytes(msg.buffer, 9);
                {
                    if (msg.message.Command == 'C') //clear errors
                    {
                        az.setError(0);
                        el.setError(0);

                        az.clear(0);
                        el.clear(0);
                        continue;
                    }

                    if (msg.message.Command == 'D') //set
                    {

                        static az_t settvals[] = {0,0,0,0};
                        static az_t sensvals[] = {0,0,0,0};

                        noInterrupts();
                        az_t azm = 0;
                        az_t elm = 0;

                        ard_st::readAzEl(msg.message.value, &azm, &elm);
                        auto ct = (counter % 2);
                        ++counter;

                        az_t curraz = az;
                        az_t currel = el;
                        interrupts();

                        settvals[ct + 0] = azm;
                        settvals[ct + 1] = elm;

                        sensvals[ct + 0] = curraz;
                        sensvals[ct + 1] = currel;

                        if (ct)
                        {
                            //have both stars defined
                            az_t err = (sensvals[2] - curraz) / (sensvals[2] - sensvals[0]);
                            az.setError(err);
                            err = (sensvals[3] - currel) / (sensvals[3] - sensvals[1]);
                            el.setError(err);
                        }
                        else
                        {
                            az0 = curraz - azm;
                            el0 = currel - elm;
                            az.setError(0);
                            el.setError(0);
                            timeElapsed = 0;
                        }
                        az.clear(azm);
                        el.clear(elm);
                        noInterrupts();
                        continue;
                    }

                    if (msg.message.Command == 'S') //set
                    {
                        az_t azm = 0;
                        az_t elm = 0;

                        ard_st::readAzEl(msg.message.value, &azm, &elm);
                        noInterrupts();

                        counter = 0;
                        az0 = az - azm;
                        el0 = el - elm;

                        az.setError(0);
                        el.setError(0);

                        az.clear(azm);
                        el.clear(elm);
                        continue;
                    }

                    if (msg.message.Command == 'R' && light > 250) //read, other part of message must be present but ignored
                    {
                        ard_st::packAzEl(msg.message.value, getAz(az - az0), el - el0);
                        Serial.write(msg.message.value.buffer, sizeof(msg.message.value.buffer));
                        Serial.flush();
                        continue;
                    }
                }
            }
            hadUsb = 0;
            interrupts();
        }

        bool static aw_once = true;
        if (abs(az.lastDelta()) > lightSens || abs(el.lastDelta()) > lightSens)
        {
            light = 0;
#ifdef USE_LCD
            analogWrite(backLightPin, MAX_BRIGHTNESS);
#endif
            aw_once = true;
        }
        int lig = light;
        interrupts(); //those asholes do not do sei() after checking timer!!

        if (lig > 30000 && hadUsb > 60000)
        {
#ifdef USE_LCD
            digitalWrite(backLightPin, LOW);
#endif
            interrupts();
            if (!sensorDataReady)
            {
                set_sleep_mode(SLEEP_MODE_IDLE); //only idle seems works by interrupt from sensor
                sleep_enable();
                sleep_mode();
                sleep_disable();
            }
        }
        else
        {
#ifdef USE_LCD
            if (lig > 7000 && aw_once)
            {
                analogWrite(backLightPin, 10);
                aw_once = false;
            }


            if (counter % 2 == 0)
            {
#endif
                if (timeElapsed > 250)
                {
                    auto t = mpu.getTemperature() / 340. + +36.53; //celsius
                    printValues(getAz(az - az0), el - el0, t);
                    timeElapsed = 0;
                }
#ifdef USE_LCD
            }
            else
            {
                if (timeElapsed < 1000)
                {
                    lcd.clear();
                    lcd.print(F("Point 2nd star &"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("press ctrl+1 again"));
                }
            }
#endif
        }
        interrupts();
    }
}