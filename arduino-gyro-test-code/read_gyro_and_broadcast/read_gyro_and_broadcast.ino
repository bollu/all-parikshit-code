//working - reads from gyro
#include <Wire.h>

//comment this line to disable debug logging
//#define DEBUG

//Serial 3:
//TX - Pin 14
//RX - Pin 15

//Serial 2:
//TX - Pin 16
//RX - Pin 17


//I2C
//SDA - 20
//SCL - 21 

//baud rate: 115200

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

void write_register(const int i2c_addr, const byte reg, const byte value) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}



void read_register(const int i2c_addr, const byte reg, byte* buffer, const int num_bytes) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(i2c_addr, num_bytes);

    int bytesread = 0;
    while(bytesread < num_bytes) {
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

//adapted from: https://courses.cs.washington.edu/courses/cse466/15au/labs/l4/MPU6050BasicExample.ino
void self_test_mpu6050(int destination[6]) {
    uint8_t raw_data[4];
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    write_register(MPU6050_I2C_ADDRESS, ACCEL_CONFIG, 0xF0);
    write_register(MPU6050_I2C_ADDRESS, GYRO_CONFIG, 0xE0);

    delay(300);  // Delay a while to let the device execute the self-test

    read_register(MPU6050_I2C_ADDRESS, SELF_TEST_X, &raw_data[0], 1);
    read_register(MPU6050_I2C_ADDRESS, SELF_TEST_Y, &raw_data[1], 1);
    read_register(MPU6050_I2C_ADDRESS, SELF_TEST_Z, &raw_data[2], 1);
    read_register(MPU6050_I2C_ADDRESS, SELF_TEST_A, &raw_data[3], 1);

    // Extract the acceleration test results first
    selfTest[0] = (raw_data[0] >> 3) | (raw_data[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (raw_data[1] >> 3) | (raw_data[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (raw_data[2] >> 3) | (raw_data[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = raw_data[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = raw_data[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = raw_data[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }
#ifdef DEBUG
    Serial.print("\nx_accel test: ");
    Serial.print(destination[0]);

    Serial.print("\ny_accel test: ");
    Serial.print(destination[1]);

    Serial.print("\nz_accel test: ");
    Serial.print(destination[2]);

    Serial.print("\nx_gyro test: ");
    Serial.print(destination[3]);

    Serial.print("\ny_gyro test: ");
    Serial.print(destination[4]);

    Serial.print("\nz_gyro test: ");
    Serial.print(destination[5]);
#endif
};


void wakeupGyro() {
    //set power to wakeup
    write_register(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
    /*Wire.beginTransmission(MPU6050_I2C_ADDRESS);
      Wire.write(MPU6050_PWR_MGMT_1);
      Wire.write(0);
      Wire.endTransmission();
      Wire.requestFrom(MPU6050_I2C_ADDRESS, 1);
     */


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

void log_string(char *string) {
#ifdef DEBUG
    Serial.print("\n");
    Serial.print(string)
#endif

}

bool g_has_gyro_woken = false;
unsigned long g_start_time = 0;


void broadcast_int16(int16_t value) {
    byte low = value;
    byte high = value >> 8;
    Serial2.write(low);
    Serial2.write(high);
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
    broadcast_int16(reading->gyrox);
    broadcast_int16(reading->gyroy);
    broadcast_int16(reading->gyroz);
    broadcast_int16(reading->accelx);
    broadcast_int16(reading->accely);
    broadcast_int16(reading->accelz);
    broadcast_int16(reading->temp);

    Serial2.flush();

}


void setup() {
    Wire.begin();
    Serial.begin(9600);
    Serial2.begin(115200);

    while(!Serial){};
    while(!Serial2){};

    g_start_time = millis();
} 

void loop() {

    if (g_has_gyro_woken && Serial2.available()) {
        byte data = Serial2.read();
        if (data != 'r') {
            Serial.print("\nunknown data from sender: ");
            Serial.print(data);
            return;
        }

        static const int TOTAL_DATA_FRAMES = 3;
        for(int i = 0; i < TOTAL_DATA_FRAMES; i++) {
            MPUReading reading = read_from_mpu_6050();
            broadcast_mpu6050_reading(&reading);
        }
    }
    else if (!g_has_gyro_woken) {
        unsigned long current_time = millis();
        unsigned long wait_to_start_seconds = 0.5;

        if ((current_time - g_start_time) >= wait_to_start_seconds * 1000) {
            wakeupGyro();
            g_has_gyro_woken = true;
            g_start_time = current_time;
            Serial.print("\ngyro woken up");
        } else {
            Serial.print("\ndelta time: ");
            Serial.print(current_time - g_start_time);
        }
    }
}
