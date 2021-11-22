
    #include "I2Cdev.h"

    #include "MPU6050_6Axis_MotionApps20.h"
    //#include "MPU6050.h" // not necessary if using MotionApps include file

    // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
    // is used in I2Cdev.h
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
    #endif

    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 mpu;
    // MPU6050 mpu(0x69); // <-- use for AD0 high

    /* =========================================================================
    NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
    depends on the MPU-6050's INT pin being connected to the Arduino's
    external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
    digital I/O pin 2.
    ========================================================================= */

    // uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
    // components with gravity removed and adjusted for the world frame of
    // reference (yaw is relative to initial orientation, since no magnetometer
    // is present in this case). Could be quite handy in some cases.
    #define OUTPUT_READABLE_WORLDACCEL

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
    }
    // ================================================================
    // ===                    MAIN PROGRAM LOOP                     ===
    // ================================================================

    //=============================================== Additional Functions
    // This is to return whether mouse is moving or not.
    bool IsStop(int v, int thresh)
    {
        return (v < thresh && v > -thresh);
    }

    #define Count 8
    int aaWorldArray[3][Count] = {0};

    int k = 0;
    float vx = 0, vy = 0, vz = 0;
    float sumx = 0, sumy = 0, sumz = 0;
    int lastx = 0, lasty = 0, lastz = 0;

    const int divider = 300 * Count; // it is to scale the values from acceleration to pixel.

    void MAF()
    {
        lastx = aaWorldArray[0][k];
        lasty = aaWorldArray[1][k];
        lastz = aaWorldArray[2][k];
        aaWorldArray[0][k] = aaWorld.x;
        aaWorldArray[1][k] = aaWorld.y;
        aaWorldArray[2][k] = aaWorld.z;
        k++;
        if (k >= Count)
            k = 0;

        sumx = sumx + aaWorld.x - lastx;
        sumy = sumy + aaWorld.y - lasty;
        sumz = sumz + aaWorld.z - lastz;
        vx = vx + sumx / divider;
        vy = vy + sumy / divider;
        vz = vz + sumz / divider;
        // Serial.print(vx);Serial.print(" ");Serial.println(vy);
    }

    void integrationF()
    {
        sumx = sumx + (float)aaWorld.x / 200;
        sumy = sumy + (float)aaWorld.y / 200;
        sumz = sumz + (float)aaWorld.z / 200;
        /*vx = vx + sumx;
        vy = vy + sumy;
        vz = vz + sumz;*/
        // Serial.print(vx);Serial.print(" ");Serial.println(vy);
    }

    //===============================================
    int buttonState = 0;
    bool lastButtonState = 0;
    bool change = 0;

    void loop()
    {
        buttonState = digitalRead(buttonPin);
        // unsigned long mytime = millis(); // it is to see sampling period
        /*while (millis() - timetime < 1000)
        {
        }
        while (millis() - timetime > 6000)
        {        digitalWrite(LED_BUILTIN, LOW);
        }
        if (millis() - timetime > 6000)
        {
            digitalWrite(LED_BUILTIN, LOW);
            buttonState = LOW;
        }else{
            buttonState = HIGH;
        }*/
        /*the led will be opened and closed according to the state of button
        The reason is to see whether there is a problem or not.*/
        if (buttonState)
        {
            digitalWrite(LED_PIN, HIGH);
        }
        else
        {
            digitalWrite(LED_PIN, LOW);
        }

        /*we set the change 1 if there is any change of the state of the button.
        if there is no any change of the state change will be 0*/
        if (buttonState != lastButtonState)
        {
            change = 1;
            lastButtonState = buttonState;
        }
        else
        {
            change = 0;
        }

        // if programming failed, don't try to do anything
        if (!dmpReady)
            return;
        // read a packet from FIFO
        // if(millis()>timetime+3000 and millis()<timetime+3500){//it is to put a time limit to run the arduino code.
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        { // Get the Latest packet // 2 to 3 ms is passed here. But the problem is that is not available always.
        // Serial.println(millis()-mytime);//it is to see the sampling period

    #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

            // MAF(); // This is the moving average filter and the integral process.
            integrationF();
            const int threshold = 40;
            /* Threshold is to set minimum movement value.
            If the values are less than threshold, means no movement*/
            float message[] = {q.x, q.y, q.z, (float)buttonState};
            Serial.write((char *)&message, sizeof(message));
            delay(10);
            /*This if else is to determine the values and whether it will send values to computer or not.
            If button is pushed or pulled, change will be 1. if the acceleration values are more than 50,
            it means there is motion. If any of this condition occurs arduino will send data to computer.
            message contains the for values. and we are using serial.write to communicate with faster speed.*/
            if (!(IsStop(aaWorld.x, threshold) && IsStop(aaWorld.y, threshold)) || change == 1 )
        {
            //int message[] = {int(sumx), int(sumy), int(sumz), buttonState};
            // int message2[] = {aaWorld.x,aaWorld.y};
            //Serial.write((char *)&message, sizeof(message));
            //Serial.print(int(sumx));Serial.print("\t");Serial.println(int(sumy));

        }  else  {
        /* This else is to make all the velocity zero and if there is no movement,
            it won't send any value to the computer*/
            sumx = sumx * 0.9;
            sumy = sumy * 0.9;
            sumz = sumz * 0.9;
        }
        
    #endif
        // Serial.println(millis()-mytime);//it is to see the sampling period
    }

    //}
    }
