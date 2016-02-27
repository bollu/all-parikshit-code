//working - reads from gyro
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define DEBUG

static const int MPU6050_I2C_ADDRESS = 0x68;
static const byte MPU6050_PWR_MGMT_1 = 0x6B;

const int GYRO_XOUT_H = 0x43; 
const int GYRO_XOUT_L = 0x44;

const int GYRO_YOUT_H = 0x45;
const int GYRO_YOUT_L = 0x46;

const int MPU6050_GYRO_ZOUT_H = 0x47;
const int MPU6050_GYRO_ZOUT_L = 0x48;

const int MPU6050_ACCEL_XOUT_H = 0x3B;
const int MPU6050_ACCEL_XOUT_L = 0x3C;

const int MPU6050_ACCEL_YOUT_H = 0x3D;
const int MPU6050_ACCEL_YOUT_L = 0x3E;

const int MPU6050_ACCEL_ZOUT_H = 0x3F;
const int MPU6050_ACCEL_ZOUT_L = 0x40;
const int MPU6050_TEMP_OUT_H = 0x41;

const int MPU6050_TEMP_OUT_L = 0x42;


const int GYRO_CONFIG = 0x1B;
const int ACCEL_CONFIG = 0x1C;

const int SELF_TEST_X = 0x0D;
const int SELF_TEST_Y = 0x0E;
const int SELF_TEST_Z = 0x0F;
const int SELF_TEST_A = 0x10;

MPU6050 mpu;


void writePin(int i2caddr, byte pin, byte *buffer, int numbytes) {

}

void read_register(const int i2caddr, const byte reg, byte* buffer, const int numbytes) {
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(i2caddr, numbytes);

  int bytesread = 0;
  while(bytesread < numbytes) {
    if (Wire.available()) {
      buffer[bytesread] = Wire.read();
    }

    bytesread++;
  }
};


int16_t read_int16(const int i2c_addr, const byte reg_high, const byte reg_low) {
    byte highval = 42, lowval = 42;
    read_register(i2c_addr, reg_high, &highval, 1);
    read_register(i2c_addr, reg_low, &lowval, 1);

    int16_t out_16bit = highval;
    out_16bit = out_16bit << 8;
    out_16bit |= lowval;

    return out_16bit;

}




void log_int16(int16_t value, const char *name) {
#ifdef DEBUG
    Serial.print("\n");
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
#endif
}

void log_double(double value, const char *name) {
#ifdef DEBUG
    Serial.print("\n");
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
#endif
}
void log_float(float value, const char *name) {
#ifdef DEBUG
    Serial.print("\n");
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
#endif
}



struct MPUReading {
    int16_t gyrox, gyroy, gyroz;
    int16_t accelx, accely, accelz;
    int16_t temp;
};


struct MPUReading read_from_mpu_6050() {
  MPUReading reading;
  reading.gyrox = read_int16(MPU6050_I2C_ADDRESS, GYRO_XOUT_H, GYRO_XOUT_L);
  reading.gyroy = read_int16(MPU6050_I2C_ADDRESS, GYRO_YOUT_H, GYRO_YOUT_L);
  reading.gyroz = read_int16(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H, MPU6050_GYRO_ZOUT_L);

  reading.accelx = read_int16(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, MPU6050_ACCEL_XOUT_L);
  reading.accely = read_int16(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H, MPU6050_ACCEL_YOUT_L);
  reading.accelz = read_int16(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H, MPU6050_ACCEL_ZOUT_L);

  reading.temp = read_int16(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H, MPU6050_TEMP_OUT_L);

  log_int16(reading.gyrox, "raw gyrox");
  log_int16(reading.gyroy, "raw gyroy");
  log_int16(reading.gyroz, "raw gyroz");
  log_int16(reading.accelx, "raw accelx");
  log_int16(reading.accely, "raw accely");
  log_int16(reading.accelz, "raw accelz");
  log_int16(reading.temp, "raw temp");
  return reading;
};


void log_mpu6050_real_readings(struct MPUReading *reading) {
  double real_gyrox = 250.0 * ((double)reading->gyrox / double(1 << 15));
  double real_gyroy = 250.0 * ((double)reading->gyroy / double(1 << 15));
  double real_gyroz = 250.0 * ((double)reading->gyroz / double(1 << 15));

  double real_accelx = 2.0 * ((double)reading->accelx / double(1 << 15));
  double real_accely = 2.0 * ((double)reading->accely / double(1 << 15));
  double real_accelz = 2.0 * ((double)reading->accelz / double(1 << 15));

  float real_temp = (reading->temp / 340.0) + 36.53;;

  log_double(real_gyrox, "gyro_x");
  log_double(real_gyroy, "gyro_y");
  log_double(real_gyroz, "gyro_z");

  log_double(real_accelx, "accel_x");
  log_double(real_accely, "accel_y");
  log_double(real_accelz, "gyro_z");

  log_float(real_temp, "temp");

}

//------------------------------------------------------------------------
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//DMP Code stuff
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void isDmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
    Serial.begin(9600);
    Wire.begin();

    while(!Serial){};

    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    while (Serial.available() && Serial.read()); // empty buffer
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //HACK - changed for UNO
        //attachInterrupt(0, isDmpDataReady, RISING);
        static const int INTERRUPT_PIN = 52;
        pinMode(INTERRUPT_PIN, INPUT);
        //pinMode(INTERRUPT_PIN, INPUT_PULLUP);
        //digitalPinToInterrupt(INTERRUPT_PIN)
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isDmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
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

  /*
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_I2C_ADDRESS, 1);

  
  int bytesRead = 0;
  while (bytesRead < 1) {
    if (Wire.available()) {
      bytesRead++;
      byte wake_state = Wire.read();
      Serial.print("\nwakeup state: ");
      Serial.print(wake_state);

    }
  }*/
}

/*
bool woken_up = false;
void loop() {

  if (Serial.available()) {
    byte data = Serial.read();

    if (data == 'w') {
      Serial.print("\nwaking up gyro...");
      wakeup();
    }
    else if (data == 'r') {
      MPUReading reading = read_from_mpu_6050();
      log_mpu6050_real_readings(&reading);
    } 
  }
}
*/

//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_GYRO
#define OUTPUT_ACCEL
//#define OUTPUT_READABLE_REALACCEL

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
          Serial.print("\nlooping, waiting for interrupt");
          delay(100);
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    //Serial.print("\nRecieved interrupt");

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_GYRO
          {
            int16_t gyro[3];
            mpu.dmpGetGyro(gyro, fifoBuffer);
            Serial.print("gyro\t");
            Serial.print(gyro[0]);
            Serial.print("\t");
            Serial.print(gyro[1]);
            Serial.print("\t");
            Serial.print(gyro[2]);
            Serial.print("\n");

          }
        #endif

        #ifdef OUTPUT_ACCEL
          {
            int16_t accel[3];
            mpu.dmpGetAccel(accel, fifoBuffer);
            Serial.print("accel\t");
            Serial.print(accel[0]);
            Serial.print("\t");
            Serial.print(accel[1]);
            Serial.print("\t");
            Serial.print(accel[2]);
            Serial.print("\n");

          }
        #endif

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
