#include <Wire.h>
#include "Adafruit_VL53L0X.h" 
#include "vl53l0x_api_core.h"
#include "Bitcraze_PMW3901.h" 

// MPU6050
const int mpu6050_addr=0x68; 
int16_t GyX,GyY,GyZ;
double throttle = 0;

// VL53L0X : 높이
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
double tHeight = 0;//target height
double cHeight = 0;//current height

// PMW3901 : 전후좌우 움직임
Bitcraze_PMW3901 flow(7);
int16_t deltaX,deltaY;
double FrcH;

void setup(){  
  Serial.begin(115200); 
  Serial1.begin(115200); 
  Wire.begin();  
  lox.begin(); 
  VL53L0X_SetRangeFractionEnable(lox.getMyDevice(), true);
  VL53L0X_set_vcsel_pulse_period(lox.getMyDevice(), 
                                  VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  VL53L0X_set_vcsel_pulse_period(lox.getMyDevice(),
                                  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);  
  flow.begin();

  Wire.beginTransmission(mpu6050_addr); 
  Wire.write(0x6b);  
  Wire.write(0);     
  Wire.endTransmission(true); 
 } 
 
void loop(){  
  Wire.beginTransmission(mpu6050_addr); 
  Wire.write(0x43);  
  Wire.endTransmission(false); 
  Wire.requestFrom(mpu6050_addr, 6, true);  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();

  static int st_bitcraze = 0;
  static int st_vl53L0X = 0;
  static bool vl53L0X_ready = false;
  static bool pwm3901_ready = false;
  if(st_bitcraze==0) {    
    if(st_vl53L0X==0)
      VL53L0X_PerformSingleMeasurement(lox.getMyDevice());//3ms  
    else if(st_vl53L0X==1) {
      VL53L0X_GetRangingMeasurementData(lox.getMyDevice(), &measure);//3ms
      vl53L0X_ready = true;
    }

    st_vl53L0X++;
    if(st_vl53L0X>1) st_vl53L0X=0;
    
  } else if(st_bitcraze==1) {
    flow.readMotionCount(&deltaX, &deltaY);//2~3ms
    pwm3901_ready = true;
  }

  st_bitcraze++;
  if(st_bitcraze>1) st_bitcraze=0;

  /*
  1. 오프셋 값 구하기
  1000번 읽어서 더해서 1000으로 나눈값 - 오프셋 값
  2. 센서 값 보정
     GyX -= OffX
  3. 각속도 구하기
     GyX /= 131 => 각속도(w)
  4. 센서 입력 주기 => dt(1/1000)
  5. 변화각도 구하기
     dQ = w*dt
  6. 현재각도 구하기
     Qnow = Qprev + dQ
   */
  static int16_t GyXOff,GyYOff,GyZOff;
  static int32_t GyXSum,GyYSum,GyZSum;
  #define NUM_SAMPLE 1000
  static int cnt_sample = NUM_SAMPLE;
  if(cnt_sample>0) {
    GyXSum += GyX;
    GyYSum += GyY;
    GyZSum += GyZ;
    cnt_sample --;
    
    if(cnt_sample==0) {
      GyXOff = GyXSum/NUM_SAMPLE;
      GyYOff = GyYSum/NUM_SAMPLE;
      GyZOff = GyZSum/NUM_SAMPLE;
    }
    delay(1);
    
    return;
  }

  // 2. 센서 값 보정
  GyX -= GyXOff;
  GyY -= GyYOff;
  GyZ -= GyZOff;

  // 3. 각속도 w 구하기 
  double GyXR, GyYR, GyZR;
  GyXR = GyX/131.0;// w
  GyYR = GyY/131.0;
  GyZR = GyZ/131.0;

  // 4. dt 구하기
  static long t_prev = 0;
  long t_now = micros();
  double dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;

  // 변화각도 구하기 d@ = w*dt
  static double AngleX, AngleY, AngleZ;
  AngleX = AngleX + GyXR*dt;
  AngleY = AngleY + GyYR*dt;
  AngleZ = AngleZ + GyZR*dt;  
  if(throttle<10) {
    AngleX = 0;
    AngleY = 0;
    AngleZ = 0;
  }
  
  // 각도오차 = 목표각도 - 현재각도
  static double tAngleX, tAngleY, tAngleZ;
  double eAngleX, eAngleY, eAngleZ;
  eAngleX = tAngleX - AngleX;
  eAngleY = tAngleY - AngleY;
  eAngleZ = tAngleZ - AngleZ;

  // 균형힘 구하기
  double FrcX, FrcY, FrcZ, Kp = 1.0;//0.5, 2.0
  FrcX = Kp * eAngleX;
  FrcY = Kp * eAngleY;
  FrcZ = Kp * eAngleZ;

  // 빠른회전 상쇄하기
  double Kd = 1.0;//2.0, 4.0, 0.5, 0.2
  FrcX = FrcX + Kd * -GyXR;
  FrcY = FrcY + Kd * -GyYR;
  FrcZ = FrcZ + Kd * -GyZR;

  // 느린 각도 접근 상쇄하기
  static double SFrcX, SFrcY, SFrcZ;
  double Ki = 1.0;
  SFrcX = SFrcX + Ki* eAngleX*dt;
  SFrcY = SFrcY + Ki* eAngleY*dt;
  SFrcZ = SFrcZ + 0* eAngleZ*dt;
  if(throttle<10) {
    SFrcX = 0;
    SFrcY = 0;
    SFrcZ = 0;
  }
  FrcX = FrcX + SFrcX;
  FrcY = FrcY + SFrcY;
  FrcZ = FrcZ + SFrcZ;

  // 사용자 입력 받기
  static uint8_t cnt_msg;
  if(Serial1.available()>0) {
    while(Serial1.available()>0) {
      uint8_t msp_data = Serial1.read();
      if(msp_data=='$') cnt_msg = 0;
      else cnt_msg++;
      if(cnt_msg==8) {//4) {//
        throttle = msp_data;//0~250cm        
        tHeight = throttle;
        if(tHeight>170) tHeight=170;
//        constrain(tHeight, 0, 170);
      }
      else if(cnt_msg==5) tAngleY = msp_data-125;
      else if(cnt_msg==6) tAngleX = -(msp_data-125);
      else if(cnt_msg==7) tAngleZ = -(msp_data-125);
      #define ANGLE_RANGE 45
      if(tAngleY<-ANGLE_RANGE) tAngleY = -ANGLE_RANGE;
      else if(tAngleY>ANGLE_RANGE) tAngleY = ANGLE_RANGE;
      if(tAngleX<-ANGLE_RANGE) tAngleX = -ANGLE_RANGE;
      else if(tAngleX>ANGLE_RANGE) tAngleX = ANGLE_RANGE;
    }
  }

  static double tAngleXPrev, tAngleYPrev; 
  static double PixelX, PixelY;  
  static double tPixelX, tPixelY;
  double ePixelX, ePixelY;

  //if(pwm3901_ready) {
  if(pwm3901_ready && tAngleX==0 && tAngleY==0) {
//  if(pwm3901_ready && tAngleX==tAngleXPrev && tAngleY==tAngleYPrev) {
    pwm3901_ready = false;

    static long t_prev_pwm3901 = 0;
    long t_now_pwm3901 = micros();
    double dt_pwm3901 = (t_now_pwm3901 - t_prev_pwm3901)/1000000.0;
    t_prev_pwm3901 = t_now_pwm3901;

    PixelX = PixelX + deltaX*dt_pwm3901;
    PixelY = PixelY + deltaY*dt_pwm3901;

    if(throttle<10) {
      PixelX = 0;
      PixelY = 0;
    }
    
    ePixelX = tPixelX - PixelX;
    ePixelY = tPixelY - PixelY;

    const double KP = 1;
    const double KD = 1.8;
    const double KI = 0.15;
    
    static double xKp = KP, yKp = KP;  
    tAngleY = -xKp*ePixelX;
    tAngleX = -yKp*ePixelY;
    
    static double xKd = KD, yKd = KD;
    tAngleY += -xKd * -deltaX;
    tAngleX += -yKd * -deltaY; 
    
    double xKi = KI, yKi = KI;
    static double SPixelX, SPixelY;  
    SPixelX += - xKi* ePixelX*dt_pwm3901;
    SPixelY += - yKi* ePixelY*dt_pwm3901;
    if(throttle<10) {
      SPixelX = 0;
      SPixelY = 0;
    }
    tAngleY += SPixelX;
    tAngleX += SPixelY;       
    if(throttle<10) {
      tAngleX = 0;
      tAngleY = 0;
    }
    
    constrain(tAngleX,-30,30);
    constrain(tAngleY,-30,30);
  }
  
//  tAngleXPrev = tAngleX;
//  tAngleYPrev = tAngleY;

  if(vl53L0X_ready) {
    vl53L0X_ready = false;
    cHeight = measure.RangeMilliMeter/10.0;//mm
//    Serial.print(measure.RangeMilliMeter);  
//    Serial.println(" mm "); 

    static long t_prev_vl53L0X = 0;
    long t_now_vl53L0X = micros();
    double dt_vl53L0X = (t_now_vl53L0X - t_prev_vl53L0X)/1000000.0;
    t_prev_vl53L0X = t_now_vl53L0X;
    
//    Serial.print(tHeight); Serial.print("|");
//    Serial.print(cHeight); Serial.print("|");
//    Serial.print(eHeight); Serial.print("|");
//    Serial.print(dt_vl53L0X); Serial.print("|");
    
    double hKp = 1.0;
    double hKd = 0.3;
    double hKi = 0.4;
    static double pHeight;    
    static double SHeight;

    double eHeight = (tHeight<=5.0)?0:(tHeight-cHeight);
    
    FrcH = hKp * eHeight;

    double dHeight = cHeight - pHeight;
    pHeight = cHeight;
    
    FrcH += hKd * -dHeight / dt_vl53L0X;
    
    SHeight += hKi * eHeight * dt_vl53L0X;
    FrcH += SHeight;

    if(throttle<10) SHeight = 0;
    if(throttle<10) FrcH = 0;
    if(FrcH<0) FrcH=0;
    else if(FrcH>250) FrcH=250;
   
//    Serial.print(dEHeight); Serial.print("|");
//    Serial.print(FrcH); Serial.println(); 
  } 

  // 모터속도 분배하기
  double SpeedA, SpeedB, SpeedC, SpeedD;
  SpeedA = FrcH/*throttle*/ + FrcZ + FrcY + FrcX;
  SpeedB = FrcH/*throttle*/ - FrcZ - FrcY + FrcX;
  SpeedC = FrcH/*throttle*/ + FrcZ - FrcY - FrcX;
  SpeedD = FrcH/*throttle*/ - FrcZ + FrcY - FrcX;
  if(throttle<10) {
    SpeedA = 0;
    SpeedB = 0;
    SpeedC = 0;
    SpeedD = 0;
  }
  if(SpeedA<0) SpeedA = 0; 
  else if(SpeedA>250) SpeedA = 250;
  if(SpeedB<0) SpeedB = 0;
  else if(SpeedB>250) SpeedB = 250;
  if(SpeedC<0) SpeedC = 0;
  else if(SpeedC>250) SpeedC = 250;
  if(SpeedD<0) SpeedD = 0;
  else if(SpeedD>250) SpeedD = 250;


  // 모터 구동
  const int motorA=6;
  const int motorB=10;
  const int motorC=9; 
  const int motorD=5;
  analogWrite(motorA, SpeedA);  
  analogWrite(motorB, SpeedB);
  analogWrite(motorC, SpeedC);
  analogWrite(motorD, SpeedD);
  
  static int cnt_loop; 
  cnt_loop++; 
  if(cnt_loop%50!=0) return; 
  
//  Serial.print(dt, 6); Serial.print("|");
//  Serial.print(1/dt, 0);
//
//  Serial.print("AnX = "); Serial.print(AngleX); 
//  Serial.print(" | AnY = "); Serial.print(AngleY); 
//  Serial.print(" | AnZ = "); Serial.print(AngleZ); 
//  Serial.print(", A = "); Serial.print(SpeedA); 
//  Serial.print(" | B = "); Serial.print(SpeedB); 
//  Serial.print(" | C = "); Serial.print(SpeedC); 
//  Serial.print(" | D = "); Serial.print(SpeedD); 

//  Serial.print("target height(cm): "); Serial.println(tHeight);
//  Serial.print("current height(cm): "); Serial.println(cHeight);
//  Serial.print("error height(cm): "); Serial.println(eHeight);

//    Serial.print("dX : ");
//    Serial.print(deltaX);
//    Serial.print("| dY: ");
//    Serial.print(deltaY);
//    Serial.print("\n");

//    Serial1.print(" | PxX = "); Serial1.print(PixelX); 
//    Serial1.print(" | PxY = "); Serial1.print(PixelY);    
//    Serial1.print("\n"); 

//    Serial1.print("AnX = "); Serial1.print(AngleX); 
//    Serial1.print(" | AnY = "); Serial1.print(AngleY); 
//
//    Serial1.print(" | tAnX = "); Serial1.print(tAngleX); 
//    Serial1.print(" | tAnY = "); Serial1.print(tAngleY);    
//    Serial1.print("\n"); 

//  Serial.println();
}
