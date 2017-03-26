#include <TinyGPS++.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include<Servo.h>
#include <Wire.h>
#include <SPI.h>
// ================================================================
// ===               DEFINE Earth values                        ===
// ================================================================
#define A 6378137.0
/* Semi-major axis */
#define ONE_F 298.257223563
/* 1/F */
#define E2  ((1.0/ONE_F)*(2-(1.0/ONE_F)))
#define NN(p) (A/sqrt(1.0 - (E2)*pow(sin(p*M_PI/180.0),2)))


//#define DEBUG_IMU
//#define DEBUG_GPS

TinyGPSPlus GPS;
MPU6050 IMU;
HMC5883L MAG;
Madgwick AHRS;
Servo FStr,PowUnit;
                                                                  /*座標系は安部先生の本に従う(右手系)*/
float roll, pitch, yaw, rollDeg, pitchDeg, yawDeg;                /*ロール、ピッチ、ヨー角(rad,deg)*/
float rollRt, pitchRt, yawRt, rollRtDeg, pitchRtDeg, yawRtDeg;    /*ロール、ピッチ、ヨーレート(rad/s,deg/s)*/
float ax, ay, az;                                                 /*X,Y,Z加速度(m/s^2)*/
float xPos,yPos,relAngle;                                         /*コース中心からの相対位置,角度*/
double heading, headingDeg;                                       /*方位(rad,deg)*/
double gpsSpeedmps;                                               /*GPS絶対速度(m/s)*/
double gpsLatDeg,gpsLonDeg;                                       /*GPS緯度・経度(deg)*/
float puPwm = 92;                                                 /*パワーユニットのPWM*/
float fStrPwm = 90;                                               /*ステアリングのPWM*/
bool started;
uint32_t timer = millis();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


// ================================================================
// ===                       Course Data                        ===
// ================================================================
float centerX;  /*コースの中心x*/
float centerY;  /*コースの中心y*/
float courseAngle; /*コースの中心XYの角度*/
float pointKeepOutX[2] = {-20,20};    /*立ち入り禁止エリア設定x(前後)方向*/
float pointKeepOutY[2] = {-10,10};    /*立ち入り禁止エリア設定y(横)方向*/
float clippingPointLat[2] = {36.5680694580f,36.5679817199f};/*コースの緯度座標*/
float clippingPointLon[2] = {139.99575805664f,139.99575805664f};/*コースの経度座標*/
float clippingPointAlt[2] = {38.6,38.6};
float clippingPointGeo[2] = {153,153};
float clippingPointX[2];
float clippingPointY[2];
//VectorFloat startLatLon[2];
//VectorFloat goalLatLon[2];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.flush();
  Serial1.begin(115200);
  IMU.setI2CMasterModeEnabled(false);
  IMU.setI2CBypassEnabled(true) ;
  IMU.setSleepEnabled(false);
  IMU.initialize();
  MAG.initialize();
  IMU.setXGyroOffset(50);
  IMU.setYGyroOffset(-35);
  IMU.setZGyroOffset(20);
  IMU.setXAccelOffset(-18);
  IMU.setYAccelOffset(-82);
  IMU.setZAccelOffset(2170);
  FStr.attach(3);
  PowUnit.attach(2);
  SetCourseData();
  delay(1000);
  //waitStartCommand();
}

void Task10ms(void)
{
    IMUupdate();
    IntegratedChassisControl();
    //GPSStrControl(0 ,90 * M_PI / 180, heading , 10  * M_PI / 180);
}

void Task100ms(void)
{
    GPSMAGupdate();
    //Serial.print(GPS.course.deg());
}

void Task1000ms(void)
{
  //Serial.println(millis());
}

void GPSMAGupdate(void)
{
  int16_t mx, my, mz;    //  MAGnetometer raw values from HMC5883L

    if(GPS.location.isUpdated() && GPS.location.isValid()){
      gpsLatDeg = GPS.location.lat();
      gpsLonDeg = GPS.location.lng();
    }
    else{
      gpsLatDeg = 0.0f;
      gpsLonDeg = 0.0f;
    }
    GPS.speed.isUpdated() && GPS.speed.isValid() ? gpsSpeedmps = GPS.speed.mps():0;
    if(gpsSpeedmps > 0.5f){
      GPS.course.isUpdated() && GPS.course.isValid() ? headingDeg = GPS.course.deg():0;
      heading = headingDeg * 0.01745329251f;
    }

#ifdef DEBUG_GPS
  Serial.print(gpsLatDeg,10);Serial.print(',');Serial.print(gpsLonDeg,11);Serial.print(',');Serial.print(heading,5);Serial.print(',');Serial.println(gpsSpeedmps);
#endif

  //Serial.print(blh2ecefx(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
  //Serial.print(",");
  //Serial.print(blh2ecefy(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
  //Serial.print(",");
  //Serial.println(blh2ecefz(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
  xPos = getRelPositionx(gpsLatDeg, gpsLonDeg, GPS.altitude.meters(), GPS.geoid.meters(), centerX, centerY, courseAngle);
  yPos = getRelPositiony(gpsLatDeg, gpsLonDeg, GPS.altitude.meters(), GPS.geoid.meters(), centerX, centerY, courseAngle);
  relAngle = heading - courseAngle;
}

void IMUupdate(void)
{
    float gx, gy, gz;    //  Gyroscope raw values from MPU-6150
    float afx,afy,afz;
    int aix, aiy, aiz;
    int gix, giy, giz;
    int agx, agy, agz;
    float sampleTime = 0.01f;
    static uint32_t lastProcessTime;

    lastProcessTime == 0 ? sampleTime = 0.01f : sampleTime = (millis() - lastProcessTime) * 0.001f;

    IMU.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

    afx = convertRawAcceleration(aiy);
    afy = -convertRawAcceleration(aix);
    afz = convertRawAcceleration(aiz);
    gx = convertRawGyro(giy);
    gy = -convertRawGyro(gix);
    gz = convertRawGyro(giz);

    AHRS.updateIMU(gx, gy, gz, afx, afy, afz);

    if(rollDeg){rollRtDeg = LimitValue((AHRS.getRoll()-rollDeg)/sampleTime,1000.0f,-1000.0f);}
    if(pitchDeg){pitchRtDeg = LimitValue((AHRS.getPitch()-pitchDeg)/sampleTime,1000.0f,-1000.0f);}
    if(yawDeg){yawRtDeg = LimitValue((AHRS.getYaw()-yawDeg)/sampleTime,1000.0f,-1000.0f);}

    rollRt = rollRtDeg * 0.01745329251f;
    pitchRt = pitchRtDeg * 0.01745329251f;
    yawRt = yawRtDeg * 0.01745329251f;

    //IMUによる方位の補正
    heading -= yawRt * sampleTime;
    headingDeg -= yawRtDeg * sampleTime;

    rollDeg = AHRS.getRoll();
    pitchDeg = AHRS.getPitch();
    yawDeg = AHRS.getYaw();

    roll = rollDeg * 0.01745329251f;
    pitch = pitchDeg * 0.01745329251f;
    yaw = yawDeg * 0.01745329251f;

    ax = convertRawAcceleration(aiy + (16384 * sin(pitch) * cos(roll)));  //重力の影響を除外した加速度x
    ay = convertRawAcceleration(-aix - (16384 * cos(pitch) * sin(roll))); //重力の影響を除外した加速度y
    az = convertRawAcceleration(aiz - (16384 * cos(pitch) * cos(roll)));  //重力の影響を除外した加速度z

    //IMUによる速度の補正
    gpsSpeedmps += ax * sampleTime;

    //IMUよる位置の補正
    xPos += gpsSpeedmps * cos(relAngle) * sampleTime;
    yPos += gpsSpeedmps * sin(relAngle) * sampleTime;

    Serial.print(xPos);Serial.print(",");Serial.print(yPos);Serial.print(",");Serial.println(relAngle);


#ifdef DEBUG_IMU
    Serial.print("Ax: ");
    Serial.print(ax);
    Serial.print(" ");
    Serial.print("Ay: ");
    Serial.print(ay);
    Serial.print(" ");
    Serial.print("Az: ");
    Serial.print(az);
    Serial.print(" ");
    Serial.print("yaw: ");
    Serial.print(yawRtDeg);
    Serial.print(" ");
    Serial.print("pitch: ");
    Serial.print(pitchRtDeg);
    Serial.print(" ");
    Serial.print("roll: ");
    Serial.println(rollRtDeg);
#endif
    lastProcessTime = millis();
}

float LimitValue(float inputValue,float upperLimitValue,float lowerLimitValue)
{
  float buf;
  buf = min(inputValue,upperLimitValue);
  buf = max(buf,lowerLimitValue);
  return buf;
}


void waitStartCommand(void)
{
  // wait for ready
  Serial.println(F("Send any character to Start RobotCar!!: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
}

void SetCourseData(void)
{
  for(uint8_t i=0;i<2;i++){
    clippingPointX[i] = blh2ecefx(clippingPointLat[i],clippingPointLon[i],clippingPointAlt[i],clippingPointGeo[i]);
    clippingPointY[i] = blh2ecefy(clippingPointLat[i],clippingPointLon[i],clippingPointAlt[i],clippingPointGeo[i]);
  }

  courseAngle = atan((clippingPointY[1]-clippingPointY[0])/(clippingPointX[1]-clippingPointX[0]));

  for(uint8_t i=0;i<2;i++){
    float clippingPointXbuf = clippingPointX[i]*cos(-courseAngle) - clippingPointY[i]*sin(-courseAngle);
    float clippingPointYbuf = clippingPointX[i]*sin(-courseAngle) + clippingPointY[i]*cos(-courseAngle);
    clippingPointX[i] = clippingPointXbuf;
    clippingPointY[i] = clippingPointYbuf;
  }

  centerX = (clippingPointX[1]-clippingPointX[0])*0.5 + clippingPointX[0];
  centerY = (clippingPointY[1]-clippingPointY[0])*0.5 + clippingPointY[0];

  for(uint8_t i=0;i<2;i++){
    clippingPointX[i] = clippingPointX[i] - centerX;
    clippingPointY[i] = clippingPointY[i] - centerY;
  }
  Serial.print("CP0:");Serial.print(clippingPointX[0]);Serial.print(",");Serial.println(clippingPointY[0]);
  Serial.print("CP1:");Serial.print(clippingPointX[1]);Serial.print(",");Serial.println(clippingPointY[1]);
  Serial.print("CenterX:");Serial.print(centerX);
  Serial.print("CenterY:");Serial.println(centerY);

}

//コース中心からの相対X距離を求める
float getRelPositionx(float lat, float lon, float alt, float geoid, float centerX, float centerY, float courseAngle)
{
  float bufx,bufy,relPosx,relPosy;
  bufx = blh2ecefx(lat,lon,alt,geoid);
  bufy = blh2ecefy(lat,lon,alt,geoid);

  relPosx = bufx*cos(-courseAngle) - bufy*sin(-courseAngle);
  relPosy = bufx*sin(-courseAngle) + bufy*cos(-courseAngle);

  relPosx -= centerX;
  relPosy -= centerY;

  return relPosx;
}

//コース中心からの相対Y距離を求める
float getRelPositiony(float lat, float lon, float alt, float geoid, float centerX, float centerY, float courseAngle)
{
  float bufx,bufy,relPosx,relPosy;
  bufx = blh2ecefx(lat,lon,alt,geoid);
  bufy = blh2ecefy(lat,lon,alt,geoid);

  relPosx = bufx*cos(-courseAngle) - bufy*sin(-courseAngle);
  relPosy = bufx*sin(-courseAngle) + bufy*cos(-courseAngle);

  relPosx -= centerX;
  relPosy -= centerY;

  return relPosy;
}


float blh2ecefx(float lat, float lon, float alt, float geoid)
{
  return (NN(lat)+(alt+geoid))*cos(lat*M_PI/180)*cos(lon*M_PI/180);
}
float blh2ecefy(float lat, float lon, float alt, float geoid)
{
  return (NN(lat)+(alt+geoid))*cos(lat*M_PI/180)*sin(lon*M_PI/180);
}
float blh2ecefz(float lat, float lon, float alt, float geoid)
{
  return (NN(lat)*(1-E2)+(alt+geoid))*sin(lat*M_PI/180);
}

/*************ICC***************/

void IntegratedChassisControl(void)
{
  if(pointKeepOutX[0] < xPos && pointKeepOutX[1] > xPos && pointKeepOutY[0] < YPos && pointKeepOutY[1] > YPos){
      //Serial.println("run");
      puPwm = 80;
    }
  else{
      //Serial.print("brake");
      BrakeCtrl(0,gpsSpeedmps,5);
  }
  PowUnit.write(puPwm);
}

/*GPS方位ベース操舵処理*/
void GPSStrControl(int directionMode, float tgtAngleRad, float nowAngleRad, float thresholdAngleRad)  //コース走行時のestAngleはクリッピングポイントから演算して入力する
{
  float limCircleMin[2],limCircleMax[2];
  float diffAngleRad;
  bool isGoStraight;

  //目標方位と現在方位差を円として表現
  //目標方位と現在方位差の許容範囲を決める(thresholdAngleRad)
  limCircleMin[0] = 1-cos(thresholdAngleRad); //円x方向最小値
  limCircleMin[1] = sin(-thresholdAngleRad);  //円y方向最小値
  limCircleMax[0] = 1;                        //円x方向最大値(演算では使用せず)
  limCircleMax[1] = sin(thresholdAngleRad);   //円y方向最大値

  //目標方位と現在方位の差
  diffAngleRad = tgtAngleRad - nowAngleRad;
  Serial.print(nowAngleRad);
  Serial.print("TurnDirection:");
  //GPS方位による直進判定
  isGoStraight = (limCircleMin[0] < cos(diffAngleRad)) &&
                 (limCircleMin[1] < sin(diffAngleRad)) &&
                 (sin(diffAngleRad) < limCircleMax[1]);

switch(directionMode){                        //旋回方向モード(-1:右,1:左,それ以外:左右)
  case -1:
    StrCtrlR(isGoStraight);
  case 1:
    StrCtrlL(isGoStraight);
  default:
    StrCtrlLR(isGoStraight,diffAngleRad);
  }
}

void StrCtrlLR(bool isGoStraight,float diffAngleRad)
{
    if(sin(diffAngleRad) >= 0){ //時計回り方向に修正した方が容易な場合
        //右に修正操舵処理
        StrCtrlR(isGoStraight);
      }
    else{                       //反時計回り方向に修正した方が容易な場合
        //左に修正操舵処理
        StrCtrlL(isGoStraight);
      }
}

void StrCtrlL(bool isGoStraight)
{
  if(isGoStraight){             //目標方位と現在方位に差がない場合
          Serial.print("GO!");
          FStr.write(90);
    }
  else{
          Serial.print("Left!");
          FStr.write(140);
    }
   Serial.println("");
}

void StrCtrlR(bool isGoStraight)
{
  if(isGoStraight){             //目標方位と現在方位に差がない場合
          Serial.print("GO!");
          FStr.write(90);
    }
  else{
          Serial.print("Right!");
          FStr.write(40);
    }
   Serial.println("");
}

uint8_t ConstTurn(bool isGoStraight, int8_t direction, float turnRadius, float targetAngle, float maxAy)
{
  float estAy = gpsSpeedmps*gpsSpeedmps/turnRadius;
  float estYawRt = maxAy/gpsSpeedmps;

  /*加減速制御*/
  /*予想旋回G,実測横GがmaxAy以上の場合は速度を下げる*/
  if(estAy > maxAy || abs(ay) > maxAy){
    puPwm += 0.5;
  }
  else if(puPwm > 90){       /*バック防止*/
    puPwm -= 0.1;
  }

  /*旋回制御*/
  if(!isGoStraight){
      if(estAy < maxAy || abs(ay) < maxAy || estYawRt < abs(yawRt)){
        fStrPwm += (float)direction * 0.5;
      }
      else if(estYawRt > abs(yawRt)){
        fStrPwm -= (float)direction * 0.2;   /*ヨーレートが過剰な場合はカウンタを当てる*/
      }
    }
  else{
    puPwm = 90;
    }
}

void BrakeCtrl(float targetSpeed, float nowSpeedmps, float maxDecelAx)
{
if(targetSpeed){
    if(nowSpeedmps > targetSpeed){
      ax > - maxDecelAx ? puPwm += 0.5 : puPwm -= 0.05 ;
    }
    else if(puPwm > 92){                     //バック防止
      puPwm = 92;
    }
  }
else{                                        //停止したい場合
  if(nowSpeedmps > 0.5){
    ax > - maxDecelAx ? puPwm += 1.0 : puPwm -= 0.05 ;
  }
  else if(puPwm > 92){                     //バック防止
    puPwm = 92;
  }
  }
 puPwm = LimitValue(puPwm,180,0);
}


float convertRawAcceleration(int aRaw) {
  // since we are using 2G(19.6 m/s^2) range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0 * 9.80665) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


void TaskMain(void)
{
  volatile uint8_t cnt10,cnt100;
  if (timer > millis()) timer = millis();
  if (millis() - timer >= 10) {
    Task10ms();
    if(cnt10>=10){
      Task100ms();
      cnt10=0;
      }
    if(cnt100>=100){
      Task1000ms();
      cnt100=0;
    }
    timer = millis();
    cnt10++;
    cnt100++;
 }
}

void loop()
{
  TaskMain();
}

void serialEvent1(){
  while (Serial1.available() > 0){
    GPS.encode(Serial1.read());
  }
}
