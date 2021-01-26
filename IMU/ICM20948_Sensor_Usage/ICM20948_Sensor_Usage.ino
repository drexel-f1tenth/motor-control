//ICM-20948 Sensor Calibration and Sensing
//Jonathan Palko
//Connect the Vcc and GND wires to 5V and GND on the Arduino board (code made for Arduino Uno architecture)
//Connect the SCL pin to the respective SCL port on the Arduino (for the Arduino Uno this is pin A5)
//Connect the SDA pin to the respective SDA port on the Arduino (for the Arduino Uno this is pin A4)
//The ICM-20948 has various settings, but the setting used here is default where 2g = 32,767, and 1g = 16,384 (the readings go plus and minus though for total range of 65,536)
//Assumed the noise was largely gaussian white noise
//Internally stored offset values don't seem accurate. So a calibration sequence was coded in during startup

//References:
//https://mjwhite8119.github.io/Robots/mpu6050#:~:text=The%20MPU6050%20is%20an%20Inertial,I2C%20bus%20for%20data%20communication.
//https://create.arduino.cc/projecthub/Nicholas_N/how-to-use-the-accelerometer-gyroscope-gy-521-6dfc19
//https://www.instructables.com/GY-521-MPU6050-3-Axis-Acceleration-Gyroscope-6DOF-/
//https://www.arduino.cc/en/Reference/Wire
//https://github.com/TKJElectronics/KalmanFilter
//https://sites.google.com/site/myimuestimationexperience/filters/complementary-filter
//https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

#include <Arduino.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <Wire.h>
#include <ICM_20948.h> //Sparkfun library for the IMU
#include <Kalman.h>    //TKJ Lauszus Kalman Filter Library 2012

// Using i2C instead of SPI since the envirnoment will be slightly noisy, and i2C has acknowledgment feature making it more robust
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when not using SPI
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0
ICM_20948_I2C myICM;   // Otherwise create an ICM_20948_I2C object
ICM_20948_AGMT_t agmt; //Create and agmt object for accel, gyro, magn, and temp data

//Booleans for activating or deactivating Digital Low Pass Filters for Accelerometer and Gyroscope
bool enaccDLPF = true;
bool engyrDLPF = false;

//Initializing the offset variables used for calibration and data collection
//Tried using integers instead, but lead to large calibration error
double accX = 0, accY = 0, accZ = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
double magX = 0, magY = 0, magZ = 0;
double OffAccX = 0, OffAccY = 0, OffAccZ = 0;
double OffGyrX = 0, OffGyrY = 0, OffGyrZ = 0;
double OffMagX = 0, OffMagY = 0, OffMagZ = 0;
double SumAccX = 0, SumAccY = 0, SumAccZ = 0;
double SumGyrX = 0, SumGyrY = 0, SumGyrZ = 0;
double SumMagX = 0, SumMagY = 0, SumMagZ = 0;
double MagX_max = 0, MagY_max = 0, MagZ_max = 0;
double MagX_min = 0, MagY_min = 0, MagZ_min = 0;

uint32_t timer;
double dt;
double dtmicro;

//Ring Buffer Variables
#if 1 //Set to 1 to enable Ring Buffer application
constexpr size_t window = 1 << 4;          //16 for sampling window columns
constexpr size_t sensors_num = 9;          //for the 6 sensors in sampling rows
double samples[sensors_num][window] = {0}; //Sample array that is 16 bits long according to window ([i][j] where i is rows and j is columns)
size_t sample_indexi = 0;                  //Index for sampling columns
size_t i = 0;                              //For initial indexing of ring buffer
double average[9][1] = {0};                //Setting average buffer to 0
#define RingBuffers
#endif

//Kalman Filter Global Variables
#if 1
double roll, pitch, yaw;
double g_roll, g_pitch, g_yaw;
double a_roll, a_pitch, a_yaw;
double mx, my, mz, Mx, My, Mz;
double norm;
Kalman KFilter[3];
Kalman KFsetting[3]; //Used for setting Qangle, Qbias, and RMeasure
double KFAngle[3];
double KFNoises[3];
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf (deals with norms)
#define KFYes
#endif

#if 1 //Set to 1 to print out timer data
#define TimeData
#endif

void setup() //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
{
    Serial.begin(115200);
    Wire.begin();
#if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
    uint64_t TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    Wire.setClock(TWBR);
#endif

    //Now connecting to the ICM-20948 sensor
    bool initialized = false; //Will be true after sensor is connected
    while (!initialized)
    {

        myICM.begin(WIRE_PORT, AD0_VAL);

        Serial.print("\r\n");
        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            SERIAL_PORT.print("Trying again...\r\n");
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }
    SERIAL_PORT.print("Device connected!\r\n");

    // Here we are doing a SW reset to make sure the device starts in a known state (software reset)
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("Software Reset returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }
    delay(250);

    // Now waking the sensor up after it was reset
    myICM.sleep(false);
    myICM.lowPower(false); //Turning off low power mode

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.
    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous (chosen here)
    //          ICM_20948_Sample_Mode_Cycled
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("setSampleMode returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }

    //Setting the Full Scale Ranges for Acc and Gyr to 2g and 250 deg/s
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm2;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                           // gpm2 (2g)
                           // gpm4 (4g)
                           // gpm8 (8g)
                           // gpm16 (16g)

    myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    // dps250 (250 deg/s)
    // dps500 (500 deg/s)
    // dps1000 (1000 deg/s)
    // dps2000 (2000 deg/s)

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("setFullScale returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }

    // Set up Digital Low-Pass Filter configuration (choose to use after it is setup)
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
                                      // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("setDLPcfg returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, enaccDLPF); //LPF for Accel
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, engyrDLPF); //LPF for Gyro
    //If not using LPF for Accel and Gyro the output rate of Accel will be 4.5kHz and Gyro will be 9kHz
    //Writes that the DLPF settings were applied correctly with no error
    SERIAL_PORT.print(F("Possible DLPF for Accelerometer returned: "));
    SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
    SERIAL_PORT.print(F("DLPF Enabled for Accelerometer returned: "));
    SERIAL_PORT.println(enaccDLPF);
    SERIAL_PORT.print(F("Possible DLPF for Gyroscope returned: "));
    SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));
    SERIAL_PORT.print(F("DLPF Enabled for Gyroscope returned: "));
    SERIAL_PORT.println(engyrDLPF);

    //Initialize the magnetometer
    myICM.startupMagnetometer();

    //Initialization and configuration for sensor is now complete
    Serial.print("\r\n");
    SERIAL_PORT.println(F("Configuration complete!"));
    Serial.print("\r\n");
    delay(100); //Just so system has the chance to get data ready for pulling

//Set to 1 in order to enable calibration
#if 1
    Serial.print("\r\n");
    SERIAL_PORT.println(F("Calibration starting. Don't move sensor!"));
    delay(50);
    Calibration(); //Run the calibration function defined below
    Serial.print("\r\n");
    SERIAL_PORT.println(F("Calibration finished. Calculated offsets shown above"));
    Serial.print("\r\n");
#endif

#ifdef KFYes
    if (myICM.dataReady())
    {
        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    }
    else
    {
        Serial.print("Waiting for data\r\n");
        delay(500);
    }

    //Initial Setting of Acceleration, Gyroscope, and Magnetometer data
    accX = (myICM.agmt.acc.axes.x - OffAccX) / 16384; //Gives acceleration in g's
    accY = (myICM.agmt.acc.axes.y - OffAccY) / 16384; //Gives acceleration in g's
    accZ = (myICM.agmt.acc.axes.z - OffAccZ) / 16384; //Gives acceleration in g's
    gyrX = (myICM.agmt.gyr.axes.x - OffGyrX) / 131;   //Gives rotational velocity in deg/s
    gyrY = (myICM.agmt.gyr.axes.y - OffGyrY) / 131;   //Gives rotational velocity in deg/s
    gyrZ = (myICM.agmt.gyr.axes.z - OffGyrZ) / 131;   //Gives rotational velocity in deg/s
    magX = (myICM.agmt.mag.axes.x - OffMagX) * 0.15;  //Gives magnetic field in microTeslas
    magY = (myICM.agmt.mag.axes.y - OffMagY) * 0.15;  //Gives magnetic field in microTeslas
    magZ = (myICM.agmt.mag.axes.z - OffMagZ) * 0.15;  //Gives magnetic field in microTeslas

// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(accY, accZ + PI) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    KFilter[0].setAngle(roll);
    KFilter[1].setAngle(pitch);

    g_roll = roll;
    g_pitch = pitch;

    norm = Norm(accX, accY, accZ);
    a_pitch = -asin(accX / norm);              // * RAD_TO_DEG;
    a_roll = asin(accY / cos(a_pitch) / norm); // * RAD_TO_DEG;

    norm = sqrt(magX * magX + magY * magY + magZ * magZ);
    mx = magX / norm;
    my = -1 * magY / norm;
    mz = magZ / norm;
    Mx = mx * cos(a_pitch) + mz * sin(a_pitch);
    My = mx * sin(a_roll) * sin(a_pitch) + my * cos(a_roll) - mz * sin(a_roll) * cos(a_pitch);
    yaw = atan2(-My, Mx) * RAD_TO_DEG;

    if (yaw > 360)
    {
        yaw -= 360;
    }
    else if (yaw < 0)
    {
        yaw += 360;
    }

    KFilter[2].setAngle(yaw);
    g_yaw = yaw;

    KFNoises[0] = KFsetting[0].getQangle();   // Process noise variance for the accelerometer
    KFNoises[1] = KFsetting[1].getQbias();    // Process noise variance for the gyro bias
    KFNoises[2] = KFsetting[2].getRmeasure(); // Measurement noise variance - this is actually the variance of the measurement noise

    //Prints out the initial noise setttings for the system
    Serial.print(KFNoises[0]);
    Serial.print("\t");
    Serial.print(KFNoises[1]);
    Serial.print("\t");
    Serial.print(KFNoises[2]);
    Serial.print("\t");
    Serial.print("\r\n");

    //For tuning the filter and setting now noise values
    KFsetting[0].setQangle(0.001);  // Process noise variance for the accelerometer
    KFsetting[1].setQbias(0.003);   // Process noise variance for the gyro bias
    KFsetting[2].setRmeasure(0.03); // Measurement noise variance - this is actually the variance of the measurement noise

    //Tuning notes:
    // The covariance matrix of process noise (Q) in this case is the accelerometer noise and the
    // estimated bias rate. This information is found from the manual of accelerometer and gyroscope
    // then round up to the close mili-unit. In addition, the covariance of measurement noise (R) is
    // referred to the variance of the measurement, if it is too high, the filter will react slowly as the
    // new measurement has more uncertainty, on the other hands, if R is too small, the output result
    // becomes noisy as we trust the new measurement of accelerometer too much

    //Pulling the new noise settings set for system
    KFNoises[0] = KFsetting[0].getQangle();   // Process noise variance for the accelerometer
    KFNoises[1] = KFsetting[1].getQbias();    // Process noise variance for the gyro bias
    KFNoises[2] = KFsetting[2].getRmeasure(); // Measurement noise variance - this is actually the variance of the measurement noise

    //Prints out the new noises settings for the system
    Serial.print(KFNoises[0]);
    Serial.print("\t");
    Serial.print(KFNoises[1]);
    Serial.print("\t");
    Serial.print(KFNoises[2]);
    Serial.print("\t");
    Serial.print("\r\n");

#endif
    timer = micros(); //Start timer for delta t calculation
}

//Simply function for calculation norm of 3 values
float Norm(float a, float b, float c)
{
    return sqrt(a * a + b * b + c * c);
}

void KalmanFilter() //------------------------------------------------------------------------------------------------------------------------------------------------------------------
{
    if (myICM.dataReady())
    {
        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    }
    else
    {
        Serial.print("Waiting for data\r\n");
        delay(500);
    }

    //Initial Setting of Acceleration, Gyroscope, and Magnetometer data
    accX = (myICM.agmt.acc.axes.x - OffAccX) / 16384; //Gives acceleration in g's
    accY = (myICM.agmt.acc.axes.y - OffAccY) / 16384; //Gives acceleration in g's
    accZ = (myICM.agmt.acc.axes.z - OffAccZ) / 16384; //Gives acceleration in g's
    gyrX = (myICM.agmt.gyr.axes.x - OffGyrX) / 131;   //Gives rotational velocity in deg/s
    gyrY = (myICM.agmt.gyr.axes.y - OffGyrY) / 131;   //Gives rotational velocity in deg/s
    gyrZ = (myICM.agmt.gyr.axes.z - OffGyrZ) / 131;   //Gives rotational velocity in deg/s
    magX = (myICM.agmt.mag.axes.x - OffMagX) * 0.15;  //Gives magnetic field in microTeslas
    magY = (myICM.agmt.mag.axes.y - OffMagY) * 0.15;  //Gives magnetic field in microTeslas
    magZ = (myICM.agmt.mag.axes.z - OffMagZ) * 0.15;  //Gives magnetic field in microTeslas

    //Updating timer
    dt = (double)(micros() - timer) / 1000000; // Calculate delta time in seconds
    dtmicro = (double)(micros() - timer);      // Calculate delta time in microseconds
    timer = micros();                          //Micros gives time in microseconds which is why 1000000 is used above

    // Reset the gyro angle when it has drifted too much
    if (gyrX < -180 || gyrX > 180)
        gyrX = roll;
    if (gyrY < -180 || gyrY > 180)
        gyrY = pitch;

        //Initial Accelerometer Calculations
        // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
        // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
        // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

//Take norm of accelerometer (not sure why we need norms here)
#if 1
    norm = Norm(accX, accY, accZ);
    a_pitch = -asin(accX / norm);              // * RAD_TO_DEG;
    a_roll = asin(accY / cos(a_pitch) / norm); // * RAD_TO_DEG;
#endif

//Take norm of magnetometer (not sure why we need norms here)
#if 1
    norm = Norm(magX, magY, magZ);
    mx = magX / norm;
    my = -1 * magY / norm; //negative 1 for sign convention correction
    mz = magZ / norm;
#endif

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && KFAngle[0] > 90) || (roll > 90 && KFAngle[0] < -90))
    {
        KFilter[0].setAngle(roll);
        KFAngle[0] = roll;
    }
    else
        KFAngle[0] = KFilter[0].getAngle(roll, gyrX, dt); // Calculate the angle using a Kalman filter

    if (abs(KFAngle[0]) > 90)
        gyrY = -gyrY; // Invert rate, so it fits the restriced accelerometer reading

    KFAngle[1] = KFilter[1].getAngle(pitch, gyrY, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && KFAngle[1] > 90) || (pitch > 90 && KFAngle[1] < -90))
    {
        KFilter[1].setAngle(pitch);
        KFAngle[1] = pitch;
    }
    else
        KFAngle[1] = KFilter[1].getAngle(pitch, gyrY, dt); // Calculate the angle using a Kalman filter

    if (abs(KFAngle[1]) > 90)
        gyrX = -gyrX;                              // Invert rate, so it fits the restriced accelerometer reading
    KFAngle[0] = kalmanX.getAngle(roll, gyrX, dt); // Calculate the angle using a Kalman filter
#endif

    //Initial Magnetometer Calculations (unneeded since it doesn't have tilt compensation)
    //float m_yaw = (float)(atan2(magY, magX)) * RAD_TO_DEG;

    //Tilt Compensation for Yaw Angle
    Mx = mx * cos(a_pitch) + mz * sin(a_pitch);
    My = mx * sin(a_roll) * sin(a_pitch) + my * cos(a_roll) - mz * sin(a_roll) * cos(a_pitch);
    yaw = atan2(-My, Mx) * RAD_TO_DEG;

    if (yaw > 360)
    {
        yaw -= 360;
    }
    else if (yaw < 0)
    {
        yaw += 360;
    }

    //Gyroscope Angle Calculations
    g_roll = gyrX * dt;
    g_pitch = gyrY * dt;
    g_yaw = gyrZ * dt;

    //ifdef RESTRICT_PITCH gives provides the adjusted roll and pitch angles
    //KFAngle[0] = KFilter[0].getAngle(roll, gyrX, dt); //angle in deg, rate in deg/s, and delta time in seconds
    //KFAngle[1] = KFilter[1].getAngle(pitch, gyrY, dt);
    KFAngle[2] = KFilter[2].getAngle(yaw, gyrZ, dt);

    float gyro_newroll = gyrX * dt;
    float gyro_newpitch = gyrY * dt;
    float gyro_newyaw = gyrZ * dt;

    //Angle summation
    g_roll += gyro_newroll;
    g_pitch += gyro_newpitch;
    g_yaw += gyro_newyaw;

    //Printing out Kalman filter data
    Serial.print(KFAngle[0]);
    Serial.print("\t");
    Serial.print(KFAngle[1]);
    Serial.print("\t");
    Serial.print(KFAngle[2]);
    Serial.print("\t");

//If you want to inclue the calculated gyro angles without filtering
#if 0
    Serial.print(g_roll);
    Serial.print("\t");
    Serial.print(g_pitch);
    Serial.print("\t");
    Serial.print(g_yaw);
    Serial.print("\t");
#endif

    Serial.print("\r\n");
}

void loop() //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
{

#ifdef KFYes
#ifdef TimeData
    Serial.print(dtmicro);
    Serial.print("\t");

    Serial.print("\t");
#endif
    KalmanFilter(); //Calling the Kalman Filter Function
#else
    dt = (double)(micros() - timer) / 1000000; // Calculate delta time in seconds
    dtmicro = (double)(micros() - timer);      // Calculate delta time in microseconds
    timer = micros();                          //Micros gives time in microseconds which is why 1000000 is used above
#endif

    if (myICM.dataReady())
    {
        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
                         //Raw data values are pulled directly from the agmt structure in 16 bit measurement units (MU for accel and gyr, microTeslas for mag)

//Printing any non Kalman Filtered data
#if 1
//Printing Data Values with Offsets (change to 0 to use different units than MU)
#if 1

#ifdef RingBuffers
        RingBuffer(); //Calling Ring Buffer values
#else
        //Updating data values with offsets (raw values)
        accX = myICM.agmt.acc.axes.x - OffAccX;
        accY = myICM.agmt.acc.axes.y - OffAccY;
        accZ = myICM.agmt.acc.axes.z - OffAccZ;
        gyrX = myICM.agmt.gyr.axes.x - OffGyrX;
        gyrY = myICM.agmt.gyr.axes.y - OffGyrY;
        gyrZ = myICM.agmt.gyr.axes.z - OffGyrZ;
        magX = myICM.agmt.mag.axes.x - OffMagX;
        magY = myICM.agmt.mag.axes.y - OffMagY;
        magZ = myICM.agmt.mag.axes.z - OffMagZ;
#endif

#ifdef TimeData
        Serial.print(dtmicro);
        Serial.print("\t");

        Serial.print("\t");
#endif
        Serial.print(accX); //Printing data values for the x-acceleration
        Serial.print("\t");
        Serial.print(accY); //Printing data values for the y-acceleration
        Serial.print("\t");
        Serial.print(accZ); //Printing data values for the z-acceleration
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(gyrX); //Printing data values for the x-gyroscope
        Serial.print("\t");
        Serial.print(gyrY); //Printing data values for the y-gyroscope
        Serial.print("\t");
        Serial.print(gyrZ); //Printing data values for the z-gyroscope
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(magX); //Printing data values for the x-magnetometer
        Serial.print("\t");
        Serial.print(magY); //Printing data values for the y-magnetometer
        Serial.print("\t");
        Serial.print(magZ); //Printing data values for the z-magnetometer
        Serial.print("\t");
        Serial.print("\r\n");

#else

#ifdef RingBuffers
        RingBuffer(); //Calling the Ring Buffer

        //Changing measurement values to m/s^2, deg/s, and microTeslas
        accX = ((myICM.agmt.acc.axes.x - OffAccX) / 16384) * 9.807; //Gives acceleration in m/s^2
        accY = ((myICM.agmt.acc.axes.y - OffAccY) / 16384) * 9.807; //Gives acceleration in m/s^2
        accZ = ((myICM.agmt.acc.axes.z - OffAccZ) / 16384) * 9.807; //Gives acceleration in m/s^2
        gyrX = (myICM.agmt.gyr.axes.x - OffGyrX) / 131;             //Gives rotational velocity in deg/s
        gyrY = (myICM.agmt.gyr.axes.y - OffGyrY) / 131;             //Gives rotational velocity in deg/s
        gyrZ = (myICM.agmt.gyr.axes.z - OffGyrZ) / 131;             //Gives rotational velocity in deg/s
        magX = (myICM.agmt.mag.axes.x - OffMagX * 0.15);            //Gives magnetometer in microTeslas
        magY = (myICM.agmt.mag.axes.y - OffMagY * 0.15);            //Gives magnetometer in microTeslas
        magZ = (myICM.agmt.mag.axes.z - OffMagZ * 0.15);            //Gives magnetometer in microTeslas
#else
        //Changing measurement values to m/s^2, deg/s, and microTeslas
        accX = ((myICM.agmt.acc.axes.x - OffAccX) / 16384) * 9.807; //Gives acceleration in m/s^2
        accY = ((myICM.agmt.acc.axes.y - OffAccY) / 16384) * 9.807; //Gives acceleration in m/s^2
        accZ = ((myICM.agmt.acc.axes.z - OffAccZ) / 16384) * 9.807; //Gives acceleration in m/s^2
        gyrX = (myICM.agmt.gyr.axes.x - OffGyrX) / 131;             //Gives rotational velocity in deg/s
        gyrY = (myICM.agmt.gyr.axes.y - OffGyrY) / 131;             //Gives rotational velocity in deg/s
        gyrZ = (myICM.agmt.gyr.axes.z - OffGyrZ) / 131;             //Gives rotational velocity in deg/s
        magX = (myICM.agmt.mag.axes.x - OffMagX * 0.15);            //Gives magnetometer in microTeslas
        magY = (myICM.agmt.mag.axes.y - OffMagY * 0.15);            //Gives magnetometer in microTeslas
        magZ = (myICM.agmt.mag.axes.z - OffMagZ * 0.15);            //Gives magnetometer in microTeslas
#endif

#ifdef TimeData
        Serial.print(dtmicro);
        Serial.print("\t");

        Serial.print("\t");
#endif
        Serial.print(accX); //Printing data values for the x-acceleration
        Serial.print("\t");
        Serial.print(accY); //Printing data values for the y-acceleration
        Serial.print("\t");
        Serial.print(accZ); //Printing data values for the z-acceleration
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(gyrX); //Printing data values for the x-gyroscope
        Serial.print("\t");
        Serial.print(gyrY); //Printing data values for the y-gyroscope
        Serial.print("\t");
        Serial.print(gyrZ); //Printing data values for the z-gyroscope
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(magX); //Printing data values for the x-magnetometer
        Serial.print("\t");
        Serial.print(magY); //Printing data values for the y-magnetometer
        Serial.print("\t");
        Serial.print(magZ); //Printing data values for the z-magnetometer
        Serial.print("\t");
        Serial.print("\r\n");

#endif
#endif
        delay(30);
    }
    else
    {
        Serial.print("Waiting for data\r\n");
        delay(500);
    }
}

void RingBuffer()
{
    /*Ring Buffer Application for things other than roll and pitch angles--------------------------------------------------------------------------------------------*/
#ifdef RingBuffers
    //Some initialization error with ring buffer since 15 points are collected and recorded before it is applied
    while (i < 15)
    {
        //Goes through and fills the first 15 columns of the samples window for averaging later
        samples[0][i] = {accX};
        samples[1][i] = {accY};
        samples[2][i] = {accZ};
        samples[3][i] = {gyrX};
        samples[4][i] = {gyrY};
        samples[5][i] = {gyrZ};
        samples[6][i] = {magX};
        samples[7][i] = {magY};
        samples[8][i] = {magZ};

        if (myICM.dataReady())
        {
            myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
            delay(30);       //Need a delay for the agmt structure to update
        }
        else
        {
            Serial.print("Waiting for data\r\n");
            delay(500);
        }

        //Need to update for first 15 data points of the ring buffer initialization
        accX = myICM.agmt.acc.axes.x - OffAccX;
        accY = myICM.agmt.acc.axes.y - OffAccY;
        accZ = myICM.agmt.acc.axes.z - OffAccZ;
        gyrX = myICM.agmt.gyr.axes.x - OffGyrX;
        gyrY = myICM.agmt.gyr.axes.y - OffGyrY;
        gyrZ = myICM.agmt.gyr.axes.z - OffGyrZ;
        magX = myICM.agmt.mag.axes.x - OffMagX;
        magY = myICM.agmt.mag.axes.y - OffMagY;
        magZ = myICM.agmt.mag.axes.z - OffMagZ;

        i++;
    }

    //Applying a ring buffer to the data (reusing defined sum variables)
    sample_indexi = 0; //Reset buffer indexing values for each time
    SumAccX = 0;
    SumAccY = 0;
    SumAccZ = 0;
    SumGyrX = 0;
    SumGyrY = 0;
    SumGyrZ = 0;
    SumMagX = 0;
    SumMagY = 0;
    SumMagZ = 0;

    //Putting the new values at the end of the new sample window
    samples[0][15] = {accX};
    samples[1][15] = {accY};
    samples[2][15] = {accZ};
    samples[3][15] = {gyrX};
    samples[4][15] = {gyrY};
    samples[5][15] = {gyrZ};
    samples[6][15] = {magX};
    samples[7][15] = {magY};
    samples[8][15] = {magZ};

    while (sample_indexi < 16) //Running sum of the 8 data points for each variable from 0 to 15
    {
        SumAccX = SumAccX + samples[0][sample_indexi];
        SumAccY = SumAccY + samples[1][sample_indexi];
        SumAccZ = SumAccZ + samples[2][sample_indexi];
        SumGyrX = SumGyrX + samples[3][sample_indexi];
        SumGyrY = SumGyrY + samples[4][sample_indexi];
        SumGyrZ = SumGyrZ + samples[5][sample_indexi];
        SumMagX = SumMagX + samples[6][sample_indexi];
        SumMagY = SumMagY + samples[7][sample_indexi];
        SumMagZ = SumMagZ + samples[8][sample_indexi];
        sample_indexi++;
    }

    //Updating the data to show only the average value of the sample window
    accX = SumAccX / 16;
    accY = SumAccY / 16;
    accZ = SumAccZ / 16;
    gyrX = SumGyrX / 16;
    gyrY = SumGyrY / 16;
    gyrZ = SumGyrZ / 16;
    magX = SumMagX / 16;
    magY = SumMagY / 16;
    magZ = SumMagZ / 16;

    //Shift the sample data for next round of filtering
    sample_indexi = 0;
    while (sample_indexi < 15)
    {
        samples[0][sample_indexi] = samples[0][sample_indexi + 1];
        samples[1][sample_indexi] = samples[1][sample_indexi + 1];
        samples[2][sample_indexi] = samples[2][sample_indexi + 1];
        samples[3][sample_indexi] = samples[3][sample_indexi + 1];
        samples[4][sample_indexi] = samples[4][sample_indexi + 1];
        samples[5][sample_indexi] = samples[5][sample_indexi + 1];
        sample_indexi++;
    }
#endif
    /*End of non roll and pitch angles Ring Buffer Application-------------------------------------------------------------------------------------------------------*/
}

//If used with data other than raw data then need to change the data being summed (currently just using raw read data)
void Calibration() //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
{
    if (myICM.dataReady())
    {
        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    }
    else
    {
        Serial.print("Waiting for data\r\n");
        delay(500);
    }

    uint8_t calp = 50; //Amount of calibration data cycles before normal operation
    uint8_t cali = 0;  //Starting point for calibration
    while (cali < calp)
    {

        //Running sum of the data over the amount of calibration cycles (averaging for accel and gyro)
        SumAccX += myICM.agmt.acc.axes.x;
        SumAccY += myICM.agmt.acc.axes.y;
        SumAccZ += myICM.agmt.acc.axes.z;
        SumGyrX += myICM.agmt.gyr.axes.x;
        SumGyrY += myICM.agmt.gyr.axes.y;
        SumGyrZ += myICM.agmt.gyr.axes.z;

        //Need mins and maxes for magnetometer calibration https://appelsiini.net/2018/calibrate-magnetometer/
        if (MagX_max < myICM.agmt.mag.axes.x)
            MagX_max = myICM.agmt.mag.axes.x;
        if (MagX_min > myICM.agmt.mag.axes.x)
            MagX_min = myICM.agmt.mag.axes.x;
        if (MagY_max < myICM.agmt.mag.axes.y)
            MagY_max = myICM.agmt.mag.axes.y;
        if (MagY_min > myICM.agmt.mag.axes.y)
            MagY_min = myICM.agmt.mag.axes.y;
        if (MagZ_max < myICM.agmt.mag.axes.z)
            MagZ_max = myICM.agmt.mag.axes.z;
        if (MagZ_min > myICM.agmt.mag.axes.z)
            MagZ_min = myICM.agmt.mag.axes.z;

#if 0 // Set to 1 to show calibration data
        Serial.print(myICM.agmt.acc.axes.x);
        Serial.print("\t");
        Serial.print(myICM.agmt.acc.axes.y);
        Serial.print("\t");
        Serial.print(myICM.agmt.acc.axes.z);
        Serial.print("\t");

        Serial.print("\t");
        Serial.print(myICM.agmt.gyr.axes.x);
        Serial.print("\t");
        Serial.print(myICM.agmt.gyr.axes.y);
        Serial.print("\t");
        Serial.print(myICM.agmt.gyr.axes.z);
        Serial.print("\t");

        Serial.print("\t");
        Serial.print(myICM.agmt.mag.axes.x);
        Serial.print("\t");
        Serial.print(myICM.agmt.mag.axes.y);
        Serial.print("\t");
        Serial.print(myICM.agmt.mag.axes.z);
        Serial.print("\t");

        Serial.print("\t");
        Serial.print("\r\n");
        delay(5);
#endif

        //Refreshing for the new accel, gyro, mag data
        if (myICM.dataReady())
        {
            myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
        }
        else
        {
            Serial.print("Waiting for data\r\n");
            delay(500);
        }

        cali += 1;
        delay(2);
    }

//Set to 1 in order to enable offsets in data collection
#if 1
    //Calculating offsets with the assumption that everything is at rest (0) during calibration (except for z-acccel due to gravity and magnetometer)
    //Positive gravity points towards ground
    //Offsets had following signs (+,+,+,-,+,-,+,+,+)
    OffAccX = SumAccX / calp;
    OffAccY = SumAccY / calp;
    OffAccZ = (SumAccZ / calp) - 16384; //Want z accel data to be at 16384
    OffGyrX = SumGyrX / calp;
    OffGyrY = SumGyrY / calp;
    OffGyrZ = SumGyrZ / calp;
    OffMagX = (MagX_max + MagX_min) / 2;
    OffMagY = (MagY_max + MagY_min) / 2;
    OffMagZ = (MagZ_max + MagZ_min) / 2;
#endif

#if 1 // Set to 1 to show offsets used with data
    Serial.print(OffAccX);
    Serial.print("\t");
    Serial.print(OffAccY);
    Serial.print("\t");
    Serial.print(OffAccZ);
    Serial.print("\t");

    Serial.print("\t");
    Serial.print(OffGyrX);
    Serial.print("\t");
    Serial.print(OffGyrY);
    Serial.print("\t");
    Serial.print(OffGyrZ);
    Serial.print("\t");

    Serial.print("\t");
    Serial.print(OffMagX);
    Serial.print("\t");
    Serial.print(OffMagY);
    Serial.print("\t");
    Serial.print(OffMagZ);
    Serial.print("\t");

    Serial.print("\t");
    Serial.print("\r\n");
    Serial.print("\r\n");
    delay(5);
#endif
}