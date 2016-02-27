//working - reads from gyro
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


//if this is defined, then output that should be written over the network (serial2)
//is written to Serial1 (logging)
static const int OUTPUT_TO_SERIAL_1 = 1;

//will only run this if OUTPUT_TO_SERIAL_! is 0
#define DEBUG

#define SERIAL_1_PRINT(x) if(!OUTPUT_TO_SERIAL_1) { Serial.print(x); };
#define SERIAL_1_PRINTLN(x) if(!OUTPUT_TO_SERIAL_1) { Serial.print(x); };

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
    if(!OUTPUT_TO_SERIAL_1) {
        #ifdef DEBUG
        SERIAL_1_PRINT("\n");
        SERIAL_1_PRINT(name);
        SERIAL_1_PRINT(": ");
        SERIAL_1_PRINT(value);
        #endif
    }
}

void log_double(double value, const char *name) {
    if(!OUTPUT_TO_SERIAL_1) {
        #ifdef DEBUG
        SERIAL_1_PRINT("\n");
        SERIAL_1_PRINT(name);
        SERIAL_1_PRINT(": ");
        SERIAL_1_PRINT(value);
        #endif
    }
}
void log_float(float value, const char *name) {
    if(!OUTPUT_TO_SERIAL_1) {
        #ifdef DEBUG
        SERIAL_1_PRINT("\n");
        SERIAL_1_PRINT(name);
        SERIAL_1_PRINT(": ");
        SERIAL_1_PRINT(value);
        #endif
    }
}



struct MPUReading {
    int16_t gyrox, gyroy, gyroz;
    int16_t accelx, accely, accelz;
    int16_t temp;
};


void broadcast_int16_serial(int16_t value) {
    byte low = value;
    byte high = value >> 8;
    Serial.write(low);
    Serial.write(high);
}

void broadcast_int16_serial2(int16_t value) {
    byte low = value;
    byte high = value >> 8;
    Serial2.write(low);
    Serial2.write(high);
}


void wakeupGyro() {
    /*
    //set power to wakeup
    write_register(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
      Wire.write(MPU6050_PWR_MGMT_1);
      Wire.write(0);
      Wire.endTransmission();
      Wire.requestFrom(MPU6050_I2C_ADDRESS, 1);
     


    Serial.print("\nwaiting for gyro wakeup response...");
    byte wakeup_state;
    read_register(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, &wakeup_state, 1);
    Serial.print("\nwoken up: ");
    Serial.print(wakeup_state == 1);

    Serial.print("\nrunning self test for gyro");    
    int test_values[6];
    self_test_mpu6050(test_values);

    Serial.print("\nconfiguring gyro settings");
    write_register(MPU6050_I2C_ADDRESS, GYRO_CONFIG, 0);
    write_register(MPU6050_I2C_ADDRESS, ACCEL_CONFIG, 0);

    byte gyro_config_state;
    read_register(MPU6050_I2C_ADDRESS, GYRO_CONFIG, &gyro_config_state, 1);
    Serial.print("\ngyro configured: ");
    Serial.print(gyro_config_state == 0);

    byte accel_config_state;
    read_register(MPU6050_I2C_ADDRESS, ACCEL_CONFIG, &accel_config_state, 1);
    Serial.print("\naccel configured: ");
    Serial.print(accel_config_state == 0);
    */
    return;
}


void broadcast_mpu6050_reading(struct MPUReading *reading) {
    #ifdef DEBUG
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
    #endif

    if(OUTPUT_TO_SERIAL_1) {
        broadcast_int16_serial(reading->gyrox);
        broadcast_int16_serial(reading->gyroy);
        broadcast_int16_serial(reading->gyroz);
        broadcast_int16_serial(reading->accelx);
        broadcast_int16_serial(reading->accely);
        broadcast_int16_serial(reading->accelz);
        broadcast_int16_serial(reading->temp);
        Serial.flush();
    }
    else {
        broadcast_int16_serial2(reading->gyrox);
        broadcast_int16_serial2(reading->gyroy);
        broadcast_int16_serial2(reading->gyroz);
        broadcast_int16_serial2(reading->accelx);
        broadcast_int16_serial2(reading->accely);
        broadcast_int16_serial2(reading->accelz);
        broadcast_int16_serial2(reading->temp);
        Serial2.flush();
    }
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


bool g_has_gyro_woken = false; //wakeup status of gyro
unsigned long g_start_time = 0; //time since gyro has started

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void isDmpDataReady() {
    mpuInterrupt = true;
}



//return read success
bool read_from_mpu_6050(struct MPUReading *reading) {
        // if programming failed, don't try to do anything
        if (!dmpReady) { 
            return false;
        }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      SERIAL_1_PRINT("\nlooping, waiting for interrupt");
      delay(10);
  }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        SERIAL_1_PRINT(F("FIFO overflow!\n"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        int16_t gyro[3];
        mpu.dmpGetGyro(gyro, fifoBuffer);
        reading->gyrox = gyro[0];
        reading->gyroy = gyro[1];
        reading->gyroz = gyro[2];

        int16_t accel[3];
        mpu.dmpGetAccel(accel, fifoBuffer);
        reading->accelx = accel[0];
        reading->accely = accel[1];
        reading->accelz = accel[2];

        reading->temp = -42;

        //broadcast the reading
        return reading;

    }
    return false;
}



void setup() {
    Serial.begin(9600);
    Serial2.begin(115200);
    Wire.begin();

    while(!Serial){};
    while(!Serial2){};

    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    while (Serial.available() && Serial.read()); // empty buffer

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //--------------
        /* FOR UNO  - DIGITAL PIN 2 for interrupt line of gyro */
        /*
        static const int UNO_INTERRUPT_PIN = 2;
        pinMode(UNO_INTERRUPT_PIN, INPUT);
        attachInterrupt(0, isDmpDataReady, RISING);
        */
        //--------------
        /* FOR DUE - DIGITAL PIN 52 for interrupt line of gyro*/
        static const int DUE_INTERRUPT_PIN = 2;
        pinMode(DUE_INTERRUPT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(DUE_INTERRUPT_PIN), isDmpDataReady, RISING);
        //---------------

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
        SERIAL_1_PRINT(F("DMP Initialization failed (code "));
        SERIAL_1_PRINT(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    g_start_time = millis();

}


void loop() {
  if (g_has_gyro_woken && Serial2.available()) {
    byte data = Serial2.read();
    if (data != 'r') {
        SERIAL_1_PRINT("\nunknown data from sender: ");
        SERIAL_1_PRINT(data);
        return;
    }

    static const int TOTAL_DATA_FRAMES = 3;
    for(int i = 0; i < TOTAL_DATA_FRAMES; i++) {
        struct MPUReading reading;

        bool has_read = read_from_mpu_6050(&reading);
        if (has_read) {
            broadcast_mpu6050_reading(&reading);

                // blink LED to indicate activity
                blinkState = !blinkState;
                digitalWrite(LED_PIN, blinkState);
            }
            else {
                SERIAL_1_PRINT("\ninvalid data from MPU6050");                
            }
        }
    }
    else if (!g_has_gyro_woken) {
        unsigned long current_time = millis();
        unsigned long wait_to_start_seconds = 0.5;

        if ((current_time - g_start_time) >= wait_to_start_seconds * 1000) {
            wakeupGyro();
            g_has_gyro_woken = true;
            g_start_time = current_time;
            SERIAL_1_PRINT("\ngyro woken up");
            } else {
                SERIAL_1_PRINT("\ndelta time: ");
                SERIAL_1_PRINT(current_time - g_start_time);
            }
        }
    }
