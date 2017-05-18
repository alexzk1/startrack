#include <avr/sleep.h>
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LCD_BACK_LIGHT_SENS_DEGREE (0.15) //sensevity of when to turn on backlight
#define SENSOR_6050_FIFO_HZ 10 //my update to motion_api, should be defined prior include
#define USE_LCD
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#include "elapsedMillis/elapsedMillis.h"
#include "my_owns/circular.h"
#include "simple_comm.h"

#ifdef USE_LCD
#include "LiquidCrystal.h"
LiquidCrystal lcd(4, 5, 10, 11, 12, 13);
#define MAX_BRIGHTNESS 15
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
//    mpu.setXGyroOffset(-63);
//    mpu.setYGyroOffset(-41);
//    mpu.setZGyroOffset(8);
//
//    mpu.setXAccelOffset(418);
//    mpu.setYAccelOffset(-622);
//    mpu.setZAccelOffset(1173);
    mpu.setXGyroOffset(-69);
    mpu.setYGyroOffset(-44);
    mpu.setZGyroOffset(12);

    mpu.setXAccelOffset(818);
    mpu.setYAccelOffset(-542);
    mpu.setZAccelOffset(1200);
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
        mpu.setDMPEnabled(true);
        // turn on the DMP, now that it's ready
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
    Serial.begin(115200);
}

static float az0  = 0;
static float el0  = 0;
static bool calibrating_once = true;
static bool clearing         = false;

static ard_st::Message msg;

float getAz(float az1)
{
    while (az1 < 0.f) az1 += M_PI * 2.f;
    return my_helpers::removeRotRad<decltype(az0)>(az1);
}

template <class I>
typename my_helpers::std::enable_if<my_helpers::std::is_integral<I>::value, VectorFloat>::type
toFloat(const Vector<I>& src)
{
    constexpr static float div = 1 << (8 * sizeof(I) - 1);
    return VectorFloat (src.x / div, src.y / div, src.z / div);
};

//---------------------------------------------------------------------------------------------------
#define twoKpDef  (2.0f * 0.5f)
#define twoKiDef  (2.0f * 0.25f)
#define betaDef	  0.2f

void MadgwickAHRSupdateIMU(float dt, Quaternion& q, float gx, float gy, float gz,  const VectorFloat& a)
{
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    Quaternion qDot{
            0.5f * (-q.x * gx - q.y * gy - q.z * gz),
            0.5f * (q.w * gx  + q.y * gz - q.z * gy),
            0.5f * (q.w * gy  - q.x * gz + q.z * gx),
            0.5f * (q.w * gz  + q.x * gy - q.y * gx)
    };

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q.w;
    _2q1 = 2.0f * q.x;
    _2q2 = 2.0f * q.y;
    _2q3 = 2.0f * q.z;
    _4q0 = 4.0f * q.w;
    _4q1 = 4.0f * q.x;
    _4q2 = 4.0f * q.y;
    _8q1 = 8.0f * q.x;
    _8q2 = 8.0f * q.y;
    q0q0 = sqrf(q.w);
    q1q1 = sqrf(q.x);
    q2q2 = sqrf(q.y);
    q3q3 = sqrf(q.z);

    // Gradient decent algorithm corrective step
    Quaternion so{
            _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y,
            _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q.x - _2q0 * a.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z,
            4.0f * q0q0 * q.y + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z,
            4.0f * q1q1 * q.z - _2q1 * a.x + 4.0f * q2q2 * q.z - _2q2 * a.y
    };
    so.normalize();
    so.scale(betaDef);
    qDot -= so;
    qDot *= dt;

    q+=qDot;
    q.normalize();
}
//---------------------------------------------------------------------------------------------------

void readSensor(my_helpers::LowPassFilter<decltype(az0)>& az, my_helpers::LowPassFilter<decltype(el0)>& el)
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
            float dte = 1.f/SENSOR_6050_FIFO_HZ;
            {
                static elapsedMillis dt = 0;
                dte += dt;
                dt = 0;
                interrupts();
            }

            for(; fifoCount >= packetSize; fifoCount -= packetSize)
            {
                static uint8_t fifoBuffer[64]; // FIFO storage buffer
                static Quaternion q;           // [w, x, y, z]         quaternion container
                static VectorFloat gravity;

                mpu.getFIFOBytes(fifoBuffer, packetSize);

                mpu.dmpGetQuaternion(msg.message.value.vals.current_quat, fifoBuffer);
                mpu.dmpGetQuaternion(&q, fifoBuffer);


                //gravity->altitude works better with not filtered values
                mpu.dmpGetGravity(&gravity, &q);
                gravity.normalize();

                const auto elt = removeRotRad<decltype(el0)>(atan(gravity.x / sqrt(sqrf(gravity.y) + sqrf(gravity.z))));
                el.push_back(elt);


                VectorInt16 tmp;
                mpu.dmpGetGyro(&tmp, fifoBuffer);
                VectorFloat gyro(toFloat(tmp));


                mpu.dmpGetAccel(&tmp, fifoBuffer);
                VectorFloat accel(toFloat(tmp));

                MadgwickAHRSupdateIMU(dte, q, gyro.x, gyro.y, gyro.z, accel);

                //https://www.reddit.com/r/Astronomy/comments/3udenf/quaternion_matrix_or_euler_angles_conversion_to/
                float azt = -2.f * atan2(q.z, q.w);
                az.push_back(getAz(azt), 0.05f);
            }
        }
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop()
{
    using az_t = decltype(az0);

    static elapsedMillis timeElapsed;
    static elapsedMillis light;
    static elapsedMillis hadUsb;
    static uint32_t counter = 0;

    static my_helpers::LowPassFilter<az_t> az(0);
    static my_helpers::LowPassFilter<az_t> el(0);

    const static float lightSens = radians(LCD_BACK_LIGHT_SENS_DEGREE);
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    if (sensorDataReady)
        readSensor(az, el);


    if (calibrating_once)
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
            calibrating_once = false;
        }
    }
    else
    {

        interrupts();
        while (Serial.available() >= 3 + 9)
        {
            hadUsb = 0;
            interrupts();
            if (Serial.find(MESSAGE_HDR))
            {
                //computer makes S/R request, and arduino just responds
                Serial.readBytes(msg.buffer, 9);
                {
                    if (msg.message.Command == 'C') //clear errors
                    {
                        counter = 0;
                        az.clear(0);
                        el.clear(0);
                        clearing = true;
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

                        az.clear(azm);
                        el.clear(elm);
                        clearing = false;
                        continue;
                    }

                    if (msg.message.Command == 'R' /*&& light > 250*/) //read, other part of message must be present but ignored
                    {
                        ard_st::packAzEl(msg.message.value, getAz(az - az0), el - el0);
                        Serial.write(msg.message.value.buffer, sizeof(msg.message.value.buffer));
                        Serial.flush();
                        continue;
                    }
                }
            }
            interrupts();
        }

        bool static aw_once = true;
        auto azld = az.lastDelta();

        if (abs(azld) > lightSens || abs(el.lastDelta()) > lightSens || clearing)
        {
            light = 0;
#ifdef USE_LCD
            analogWrite(backLightPin, MAX_BRIGHTNESS);
#endif
            aw_once = true;
        }
        int lig = light;
        interrupts(); //those asholes do not do sei() after checking timer!!

        if (lig > 20000 && hadUsb > 60000)
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

                analogWrite(backLightPin, 5);
                aw_once = false;
            }
#endif
            if (timeElapsed > 250)
            {
                auto t = mpu.getTemperature() / 340. + +36.53; //celsius
                printValues(getAz(az - az0), el - el0, t);
                timeElapsed = 0;
            }
        }
        interrupts();
    }
}