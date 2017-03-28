#include <TinyGPS++.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include <Servo.h>
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
#define DEBUG_GPS

TinyGPSPlus GPS;
MPU6050 IMU;
HMC5883L MAG;
Madgwick AHRS;
Servo FStr,PowUnit;
                                                                  /*座標系は安部先生の本に従う(右手系)*/
VectorFloat rpyAngle;
VectorFloat rpyRate;
VectorFloat acc;                                                  /*X,Y,Z加速度(m/s^2)*/
VectorFloat spd;
VectorFloat dist;
VectorFloat pos2D;
float relAngle;                                                   /*コース中心からの相対位置,角度*/
double heading, headingDeg;                                       /*方位(rad,deg)*/
double gpsSpeedmps;                                               /*GPS絶対速度(m/s)*/
polVectorFloat3D gpsLatLon;                                       /*GPS緯度・経度(deg)*/
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
VectorFloat center;
float courseAngle;                  /*コースの中心XYの角度*/
VectorFloat pointKeepOut[2];        /*立ち入り禁止エリア設定*/
polVectorFloat3D clippingPoint[2];
float clippingPointAlt[2];
float clippingPointGeo[2];
VectorFloat clippingPoint2D[2];
//VectorFloat startLatLon[2];
//VectorFloat goalLatLon[2];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Wire.begin();
  Serial.begin(115200);
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
  Serial.flush();
  waitStartCommand();
}

void Task10ms(void)
{
    IMUupdate();
    GPSupdate();
    IntegratedChassisControl();
    //GPSStrControl(0 ,90 * M_PI / 180, heading , 10  * M_PI / 180);
}

void Task100ms(void)
{

    //Serial.print(GPS.course.deg());
}

void Task1000ms(void)
{
  //Serial.println(millis());
}

void GPSupdate(void)
{
    float sampleTime = 0.01f;
    static uint32_t lastProcessTime;
    lastProcessTime == 0 ? sampleTime = 0.01f : sampleTime = (millis() - lastProcessTime) * 0.001f;

    if(GPS.speed.isUpdated() && GPS.speed.isValid()){
        gpsSpeedmps = GPS.speed.mps();
      }
    else{//GPS情報受信・更新できない間は縦加速度による速度補正
        gpsSpeedmps += acc.x * sampleTime;
      }
    if(gpsSpeedmps > 0.5f && GPS.course.isUpdated() && GPS.course.isValid()){
        heading = GPS.course.rad();
      }
    else{//GPS情報受信・更新できない・速度が低い間はヨーレートによる方位補正
        float headingbuf;
        headingbuf  = heading;
        headingbuf -= rpyRate.z * sampleTime;
        heading     = RoundRad(headingbuf);
      }
    headingDeg = heading * 180 / M_PI;
    relAngle = RoundRad(heading - courseAngle);
    //FPSの緯度経度が受信・更新できた場合
    if(GPS.location.isUpdated() && GPS.location.isValid()){
        gpsLatLon.t = GPS.location.lat();
        gpsLatLon.p = GPS.location.lng();
        pos2D = getRelPosition(gpsLatLon, GPS.altitude.meters(), GPS.geoid.meters(), center, courseAngle);
      }
    //現在、GPSの緯度経度の受信・更新できないが、前回値が入力されている場合は速度・相対方位情報で位置修正
    else if(gpsLatLon.t && gpsLatLon.p){
        pos2D.x += gpsSpeedmps * cos(relAngle) * sampleTime;
        pos2D.y += gpsSpeedmps * sin(relAngle) * sampleTime;
      }
    else{//現在、GPSの緯度経度の受信・更新できないが、GPSの緯度経度が初期値の場合は受信するまで待つ
        pos2D = getRelPosition(gpsLatLon, GPS.altitude.meters(), GPS.geoid.meters(), center, courseAngle);
    }
#ifdef DEBUG_GPS
Serial.print("Lat:");Serial.print(gpsLatLon.t);Serial.print("Lon:");Serial.print(gpsLatLon.p);
Serial.print("Alt:");Serial.print(GPS.altitude.meters());Serial.print("Geo:");Serial.print(GPS.geoid.meters());
Serial.print("PosX:");Serial.print(pos2D.x);Serial.print("PosY:");Serial.print(pos2D.y);Serial.print("RelAngle:");
Serial.print(relAngle);Serial.print("Speed:");Serial.println(gpsSpeedmps);
#endif
  //Serial.print(blh2ecefx(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
  //Serial.print(",");
  //Serial.print(blh2ecefy(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
  //Serial.print(",");
  //Serial.println(blh2ecefz(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));

  lastProcessTime = millis();

}

void IMUupdate(void)
{
    float gfx, gfy, gfz;    //  Gyroscope raw values from MPU-6150
    float afx,afy,afz;
    int aix, aiy, aiz;
    int gix, giy, giz;
    float sampleTime = 0.01f;
    static uint32_t lastProcessTime;
    VectorFloat rpyAngleDeg;

    lastProcessTime == 0 ? sampleTime = 0.01f : sampleTime = (millis() - lastProcessTime) * 0.001f;

    IMU.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

    afx = convertRawAcceleration(aiy);
    afy = -convertRawAcceleration(aix);
    afz = convertRawAcceleration(aiz);
    gfx = convertRawGyro(giy);
    gfy = -convertRawGyro(gix);
    gfz = convertRawGyro(giz);

    AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);

    if(rpyAngle.calcRad2Deg().x){rpyRate.x = LimitValue((AHRS.getRoll()-rpyAngle.calcRad2Deg().x)/sampleTime,1000.0f,-1000.0f) * M_PI/180;}
    if(rpyAngle.calcRad2Deg().y){rpyRate.y = LimitValue((AHRS.getPitch()-rpyAngle.calcRad2Deg().y)/sampleTime,1000.0f,-1000.0f) * M_PI/180;}
    if(rpyAngle.calcRad2Deg().z){rpyRate.z = LimitValue((AHRS.getYaw()-rpyAngle.calcRad2Deg().z)/sampleTime,1000.0f,-1000.0f) * M_PI/180;}

    rpyAngle.x = AHRS.getRoll() * M_PI/180;   //ロール角
    rpyAngle.y = AHRS.getPitch() * M_PI/180;  //ピッチ角
    rpyAngle.z = AHRS.getYaw() * M_PI/180;    //ヨー角

    acc.x = convertRawAcceleration(aiy + (16384 * sin(rpyAngle.y) * cos(rpyAngle.x)));  //重力の影響を除外した加速度x
    acc.y = convertRawAcceleration(-aix - (16384 * cos(rpyAngle.y) * sin(rpyAngle.x))); //重力の影響を除外した加速度y
    acc.z = convertRawAcceleration(aiz - (16384 * cos(rpyAngle.y) * cos(rpyAngle.x)));  //重力の影響を除外した加速度z


#ifdef DEBUG_IMU
    Serial.print("Ax: ");
    Serial.print(acc.x);
    Serial.print(" ");
    Serial.print("Ay: ");
    Serial.print(acc.y);
    Serial.print(" ");
    Serial.print("Az: ");
    Serial.print(acc.z);
    Serial.print(" ");
    Serial.print("roll: ");
    Serial.print(rpyAngle.x);
    Serial.print(" ");
    Serial.print("pitch: ");
    Serial.print(rpyAngle.y);
    Serial.print(" ");
    Serial.print("yaw: ");
    Serial.println(rpyAngle.z);
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
  /*********コースの座標を入力*********/
  clippingPoint[0].t = 36.5680694580f;    clippingPoint[1].t = 36.5679817199f;    /*緯度設定*/
  clippingPoint[0].p = 139.99575805664f;  clippingPoint[1].p = 139.99575805664f;  /*経度設定*/

  clippingPointAlt[0] = 153;  clippingPointAlt[1] = 153;        /*標高設定*/
  clippingPointGeo[0] = 38.6; clippingPointGeo[1] = 38.6;       /*ジオイド高設定*/
  /*******立ち入り禁止エリア設定*******/
  pointKeepOut[0].x = -20;  pointKeepOut[1].x = 20;            /*立ち入り禁止エリア設定x(前後)方向*/
  pointKeepOut[0].y = -20;  pointKeepOut[1].y = 20;            /*立ち入り禁止エリア設定y(横)方向*/
  /********************************/
  VectorFloat buf0[2],buf1[2];

  for(uint8_t i=0;i<2;i++){
    buf0[i] = blh2ecef(clippingPoint[i],clippingPointAlt[i],clippingPointGeo[i]);
    }

  courseAngle = atan((buf0[1].y-buf0[0].y)/(buf0[1].x-buf0[0].x));

  for(uint8_t i=0;i<2;i++){
    buf1[i].x = buf0[i].x * cos(-courseAngle) - buf0[i].y * sin(-courseAngle);
    buf1[i].y = buf0[i].x * sin(-courseAngle) + buf0[i].y * cos(-courseAngle);
    }

  center.x = (buf1[1].x-buf1[0].x)*0.5 + buf1[0].x;
  center.y = (buf1[1].y-buf1[0].y)*0.5 + buf1[0].y;

  for(uint8_t i=0;i<2;i++){
    clippingPoint2D[i].x = buf1[i].x - center.x;
    clippingPoint2D[i].y = buf1[i].y - center.y;
  }

  Serial.print("CP0:");Serial.print(clippingPoint2D[0].x);Serial.print(",");Serial.println(clippingPoint2D[0].y);
  Serial.print("CP1:");Serial.print(clippingPoint2D[1].x);Serial.print(",");Serial.println(clippingPoint2D[1].y);
  Serial.print("CenterX:");Serial.print(center.x);
  Serial.print("CenterY:");Serial.println(center.y);

}

//コース中心からの相対距離を2D直交座標系で求める
VectorFloat getRelPosition(polVectorFloat3D LatLon, float alt, float geoid, VectorFloat center, float courseAngle)
{
  VectorFloat buf0,relPos2D;
  buf0 = blh2ecef(LatLon,alt,geoid);

  relPos2D.x = buf0.x*cos(-courseAngle) - buf0.y*sin(-courseAngle);
  relPos2D.y = buf0.x*sin(-courseAngle) + buf0.y*cos(-courseAngle);

  relPos2D.x -= center.x;
  relPos2D.y -= center.y;

  //Serial.print(relPos2D.x);Serial.print(",");Serial.println(relPos2D.y);

  return relPos2D;
}

VectorFloat blh2ecef(polVectorFloat3D LatLon, float alt, float geoid)
{
  VectorFloat ecef;
  ecef.x = (NN(LatLon.t)+(alt+geoid))*cos(LatLon.t*M_PI/180)*cos(LatLon.p*M_PI/180);
  ecef.y = (NN(LatLon.t)+(alt+geoid))*cos(LatLon.t*M_PI/180)*sin(LatLon.p*M_PI/180);
  ecef.z = (NN(LatLon.t)*(1-E2)+(alt+geoid))*sin(LatLon.t*M_PI/180);
  return ecef;
}

/*************ICC***************/

void IntegratedChassisControl(void)
{
  if(pointKeepOut[0].x < pos2D.x && pointKeepOut[1].x > pos2D.x && pointKeepOut[0].y < pos2D.y && pointKeepOut[1].y > pos2D.y){
      //Serial.println("run");
      //puPwm = 80;
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
  if(estAy > maxAy || abs(acc.y) > maxAy){
    puPwm += 0.5;
  }
  else if(puPwm > 90){       /*バック防止*/
    puPwm -= 0.1;
  }

  /*旋回制御*/
  if(!isGoStraight){
      if(estAy < maxAy || abs(acc.y) < maxAy || estYawRt < abs(rpyRate.z)){
        fStrPwm += (float)direction * 0.5;
      }
      else if(estYawRt > abs(rpyRate.z)){
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
      acc.x > - maxDecelAx ? puPwm += 0.5 : puPwm -= 0.05 ;
    }
    else if(puPwm > 92){                     //バック防止
      puPwm = 92;
    }
  }
else{                                        //停止したい場合
  if(nowSpeedmps > 0.5){
    acc.x > - maxDecelAx ? puPwm += 1.0 : puPwm -= 0.05 ;
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

float RoundRad(float x) //角度を表す変数を0～6.28rad(2 * M_PI)の範囲に収める関数
{
	if (x >= 0.f) {
		return fmod(x, 2 * M_PI);
	} else {
		return 2 * M_PI - fmod(-x, 2 * M_PI);
	}
	return x;
}

float RoundDeg(float x) //角度を表す変数を0～360degの範囲に収める関数
{
	if (x >= 0.f) {
		return fmod(x, 360.f);
	} else {
		return 360.f - fmod(-x, 360.f);
	}
	return x;
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
