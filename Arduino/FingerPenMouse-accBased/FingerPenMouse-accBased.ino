
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_QUATERNION

#define MOUSEEVENTF_MOVE 0x0001
#define MOUSEEVENTF_LEFTDOWN 0x0002
#define MOUSEEVENTF_LEFTUP 0x0004
#define MOUSEEVENTF_RIGHTDOWN 0x0008
#define MOUSEEVENTF_RIGHTUP 0x0010
#define MOUSEEVENTF_MIDDLEDOWN 0x0020
#define MOUSEEVENTF_MIDDLEUP 0x0040

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

/*  =========================================================================
    NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
    depends on the MPU-6050's INT pin being connected to the Arduino's
    external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
    digital I/O pin 2.
    ========================================================================= */

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.


#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
unsigned long timetime;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    /*  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
    */
    // load and configure the DMP
    //    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-205);
    mpu.setYGyroOffset(-101);
    mpu.setZGyroOffset(35);
    mpu.setZAccelOffset(1553); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        //        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        digitalPinToInterrupt(INTERRUPT_PIN);
        //        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        // Serial.println("Serialbegin");
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(250000);
#define buttonPin 3
    pinMode(buttonPin, INPUT);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    timetime = millis(); // it is to put a time limit to run the arduino code.

    byte message = {85};//to start the communication.
    Serial.write((char *)&message, sizeof(message));
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

//=============================================== Additional Functions
// This is to return whether mouse is moving or not.

int k = 0;
float xpos = 0;
float ypos = 0;
float zpos = 0;
float vx = 0;
float vy = 0;
float vz = 0;
//===============================================
bool btn = 0;
int buttonState = MOUSEEVENTF_LEFTDOWN;
int lastButtonState = buttonState;


void loop()
{
    btn = digitalRead(buttonPin);
    // unsigned long mytime = millis(); // it is to see sampling period
    /*  while (millis() - timetime < 1000)
        {
        }
        while (millis() - timetime > 6000)
        {        digitalWrite(LED_BUILTIN, LOW);
        }
        if (millis() - timetime > 6000)
        {
        digitalWrite(LED_BUILTIN, LOW);
        btn = LOW;
        }else{
        btn = HIGH;
        }*/
    /*  the led will be opened and closed according to the state of button
        The reason is to see whether there is a problem or not.*/
    if (btn) {
        buttonState = MOUSEEVENTF_LEFTDOWN;
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
        buttonState = MOUSEEVENTF_LEFTUP;
    }

    /*  we set the change 1 if there is any change of the state of the button.
        if there is no any change of the state change will be 0*/
    if (buttonState != lastButtonState) {
        lastButtonState = buttonState;
        int message[] = {(int)buttonState, 0, 0, 0};
        Serial.write((char *)&message, sizeof(message));
    }

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    // if(millis()>timetime+3000 and millis()<timetime+3500){//it is to put a time limit to run the arduino code.
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {   // Get the Latest packet // 2 to 3 ms is passed here. But the problem is that is not available always.
        // Serial.println(millis()-mytime);//it is to see the sampling period

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    float accel_a =(float) aaReal.x, accel_b = (float)aaReal.y, accel_c = -(float)aaReal.z;
    #define accel_correction 1
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    float accel_a = (float)aaWorld.x, accel_b = (float)aaWorld.y, accel_c = -(float)aaWorld.z;
    #define accel_correction 1/131070
#endif

#define sensitivity 10 // angle correction is to equalize each angle
#define pixel_normalizer 10000 
#define correction_term sensitivity/ pixel_normalizer
        vx = 0.98* vx + accel_a * accel_correction;// * dt;
        vy = 0.98* vy + accel_b * accel_correction;// * dt;
        vz = 0.98* vz + accel_c * accel_correction;// * dt;
        
        int dx = (int)(vx *correction_term);
        int dy = (int)(vy *correction_term);
        int dz = (int)(vz *correction_term);

        if (dx != 0 || dy != 0 || dz != 0) {
            int message[] = {(int) MOUSEEVENTF_MOVE, dx, dz, dy};
            Serial.write((char *)&message, sizeof(message));
            delay(10);
        }
        /*  Serial.print("q: x: ");
            Serial.print(q.x, 5);
            Serial.print("\ty: ");
            Serial.print(q.y, 5);
            Serial.print("\tz: ");
            Serial.print(q.z, 5);
            Serial.print("\tw: ");
            Serial.print(q.w, 5);
            Serial.print("\tpsi: ");
            Serial.print(euler[0], 5);
            Serial.print("\ttheta: ");
            Serial.print(euler[1], 5);
            Serial.print("\tphi: ");
            Serial.print(euler[2], 5);
            Serial.print("\ty: ");
            Serial.print(ypr[0], 5);
            Serial.print("\tp: ");
            Serial.print(ypr[1], 5);
            Serial.print("\tr: ");
            Serial.print(ypr[2], 5);
            Serial.print("\n");*/


        // Serial.println(millis()-mytime);//it is to see the sampling period
    }

}
