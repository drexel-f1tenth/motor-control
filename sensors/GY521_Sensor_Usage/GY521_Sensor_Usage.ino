//Jonathan Palko
//Possibly going to add a ring buffer rolling average to improve sensor accuracy
//References: 
//https://mjwhite8119.github.io/Robots/mpu6050#:~:text=The%20MPU6050%20is%20an%20Inertial,I2C%20bus%20for%20data%20communication. 
//https://create.arduino.cc/projecthub/Nicholas_N/how-to-use-the-accelerometer-gyroscope-gy-521-6dfc19

#include<Wire.h>
#include <math.h>
const int MPU=0x68;
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
double pitch,roll;

 void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
  }

  int AcXoff,AcYoff,AcZoff,GyXoff,GyYoff,GyZoff;
  
  size_t cal_data = 25;
  size_t cal_start = 0;
  size_t i = 0;
  
  long Sum_Xa = 0;
  long Sum_Ya = 0;
  long Sum_Za = 0;
  
  long Sum_Xg = 0;
  long Sum_Yg = 0;
  long Sum_Zg = 0;

void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);

  //Calibration Calculations
  if(i < cal_data){
    delay(30);
      
    //read accel data
    AcX=(Wire.read()<<8|Wire.read());
    AcY=(Wire.read()<<8|Wire.read());
    AcZ=(Wire.read()<<8|Wire.read());
    
    Sum_Xa = Sum_Xa + AcX;
    Sum_Ya = Sum_Ya + AcY;
    Sum_Za = Sum_Za + AcZ;
    
    //read gyro data
    GyX=(Wire.read()<<8|Wire.read());
    GyY=(Wire.read()<<8|Wire.read());
    GyZ=(Wire.read()<<8|Wire.read());
    
    Sum_Xg = Sum_Xg + GyX;
    Sum_Yg = Sum_Yg + GyY;
    Sum_Zg = Sum_Zg + GyZ;
    
    Serial.print("Accelerometer: ");
    Serial.print("X = "); 
    Serial.print(AcX);
    Serial.print(" | Y = "); 
    Serial.print(AcY);
    Serial.print(" | Z = "); 
    Serial.println(AcZ);
    
    Serial.print("Gyroscope: ");
    Serial.print("X = "); 
    Serial.print(GyX);
    Serial.print(" | Y = "); 
    Serial.print(GyY);
    Serial.print(" | Z = "); 
    Serial.println(GyZ);
    Serial.println(" ");
    
    i++;
  }else{
    if(cal_start == 0){
        //Calibrating the accelerometer offsets
        AcXoff = (double) Sum_Xa/cal_data;
        AcYoff = (double) Sum_Ya/cal_data;
        AcZoff = (double) Sum_Za/cal_data;
        
        //Calibrating the gyroscope offsets
        GyXoff = (double) Sum_Xg/cal_data;
        GyYoff = (double) Sum_Yg/cal_data;
        GyZoff = (double) Sum_Zg/cal_data;
        
        Serial.println("Calibration Values: ");
        Serial.print("AcX = "); 
        Serial.print(AcXoff);
        Serial.print("  AcY = "); 
        Serial.print(AcYoff);
        Serial.print("  AcZ = "); 
        Serial.println(AcZoff);
        
        Serial.print("GyX = "); 
        Serial.print(GyXoff);
        Serial.print("  GyY = "); 
        Serial.print(GyYoff);
        Serial.print("  GyZ = "); 
        Serial.println(GyZoff);
        Serial.println(" ");
        
        cal_start = 1;
    }
        //read accel data
        AcX=(Wire.read()<<8|Wire.read()) - AcXoff;
        AcY=(Wire.read()<<8|Wire.read()) - AcYoff;
        AcZ=(Wire.read()<<8|Wire.read()) + 16384 - AcZoff; //@g is equivalent to 16384
        
        //read gyro data
        GyX=(Wire.read()<<8|Wire.read()) - GyXoff;
        GyY=(Wire.read()<<8|Wire.read()) - GyYoff;
        GyZ=(Wire.read()<<8|Wire.read()) - GyZoff;
        
        Serial.print("Accelerometer: ");
        Serial.print("X = "); 
        Serial.print(AcX);
        Serial.print(" | Y = "); 
        Serial.print(AcY);
        Serial.print(" | Z = "); 
        Serial.println(AcZ);
        
        Serial.print("Gyroscope: ");
        Serial.print("X = "); 
        Serial.print(GyX);
        Serial.print(" | Y = "); 
        Serial.print(GyY);
        Serial.print(" | Z = "); 
        Serial.println(GyZ);
        Serial.println(" ");
        delay(30);
      }
    }
