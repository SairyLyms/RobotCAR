/************************************************************************/
/*	Include Files														*/
/************************************************************************/
#include <NMEAGPS.h>
#include <TinyGPS++.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <TaskScheduler.h>
#include <Filters.h>
/************************************************************************/
/*	Macro Definition													*/
/************************************************************************/
/************************************************************************/
/*	Struct Definition													*/
/************************************************************************/
/************************************************************************/
/*	Private Constant Definition											*/
/************************************************************************/


// ================================================================
// ===                       Course Data                        ===
// ================================================================
VectorFloat center;
polVectorFloat3D distToCP[2];
polVectorFloat3D clippingPoint[2];
VectorFloat pointKeepOut[2];        /*立ち入り禁止エリア設定*/

//#define DEBUG_IMU
#define DEBUG_GPS
//#define DEBUG
//#define TechCom											/*テストコース設定*/
#ifndef TechCom
#define Ground
//#define Garden
//#define HappiTow
#endif
#ifdef TechCom
NeoGPS::Location_t cp1(365759506L,1400158539L);
NeoGPS::Location_t cp2(365760193L,1400160370L);
#elif defined Garden
NeoGPS::Location_t cp1(365780250L,1400148840L);
NeoGPS::Location_t cp2(365780740L,1400150240L);
#elif defined Ground
NeoGPS::Location_t cp1(365835260L,1400109670L);
NeoGPS::Location_t cp2(365833660L,1400110270L);
#elif defined HappiTow
NeoGPS::Location_t cp1(365679436L,1399957886L);
NeoGPS::Location_t cp2(365680389L,1399957886L);
#endif
NeoGPS::Location_t cp[2] = {cp1,cp2};
NeoGPS::Location_t locCenter(0.5*(cp1.lat()+cp2.lat()),0.5*(cp1.lon()+cp2.lon()));
NeoGPS::Location_t relPos2D;
static NMEAGPS  gps;
static gps_fix  fix;
MPU6050 IMU;
HMC5883L MAG;
Madgwick AHRS;
Servo FStr,PowUnit;
Scheduler runner;
/*座標系は安部先生の本に従う(右手系)*/
VectorFloat rpyAngle;
VectorFloat rpyRate;
VectorFloat acc;                                                  /*X,Y,Z加速度(m/s^2)*/
float relHeading,relYawAngle; 																		/*CP間中心線からの相対角度(Heading:CW+,Yaw:CCW+)*/
float relHeadingTgt[2],relYawAngleTgt[2];													/*CPまでの相対位置,角度(Heading:CW+,Yaw:CCW+)*/
float heading, headingDeg;																				/*方位(rad,deg)*/
float headingOffst = cp[0].BearingTo(cp[1]);											/*CP間の方位*/
float gpsSpeedmps;                                               /*GPS絶対速度(m/s)*/
polVectorFloat3D centerPos;
int puPwm = 92;                                                 /*パワーユニットのPWM*/
int fStrPwm = 90;                                               /*ステアリングのPWM*/
bool started;

// ================================================================
// ===               				Prototype Def.			                ===
// ================================================================
void Task10ms(void);
void Task20ms(void);
void Task100ms(void);
void Task1000ms(void);
// ================================================================
// ===               						Tasks						                ===
// ================================================================
Task T10(10, TASK_FOREVER, &Task10ms, &runner,true);
Task T20(20, TASK_FOREVER, &Task20ms, &runner,true);
Task T100(100, TASK_FOREVER, &Task100ms, &runner,true);
Task T1000(1000, TASK_FOREVER, &Task1000ms, &runner,true);
// ================================================================
// ===                       Course Variable                    ===
// ================================================================
float courseAngle;                  /*コースの中心XYの角度*/
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
	Serial1.begin(38400);
	IMU.setI2CMasterModeEnabled(false);
	IMU.setI2CBypassEnabled(true);
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
	Serial.flush();
	runner.startNow();
	Initwait();
}

/************************************************************************
 * FUNCTION : 10 ms周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task10ms(void)
{
	IMUupdate();
	IntegratedChassisControl();
}

/************************************************************************
 * FUNCTION : 20 ms周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task20ms(void)
{
	ChassisFinalOutput(puPwm,fStrPwm);
}

/************************************************************************
 * FUNCTION : 100 ms周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task100ms(void)
{
	//Serial.print(GPS.course.deg());
}

/************************************************************************
 * FUNCTION : 1000 ms周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task1000ms(void)
{
	//Serial.println(millis());
}

/************************************************************************
 * FUNCTION : GPS更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void GPSupdate(void)
{
	static uint8_t sats;
	fix = gps.read();
	if(fix.valid.time){
	}
	if(fix.valid.location) {
		NeoGPS::Location_t cpAway[2] = {cp[0],cp[1]};
		for(int8_t i=0;i<2;i++){
			cpAway[i].OffsetBy( 10.0 / NeoGPS::Location_t::EARTH_RADIUS_KM, fix.location.BearingTo(cp[i]));
			distToCP[i].r = fix.location.DistanceKm(cp[i]) * 1000.0f;
			distToCP[i].t = fix.location.BearingTo(cpAway[i]); //直進時制御用のリファレンス方位
			distToCP[i].p = fix.location.BearingTo(cp[i]); //旋回開始判定用の方位
			}
			centerPos.r = locCenter.DistanceKm(fix.location) * 1000.0f;
			centerPos.t = locCenter.BearingTo(fix.location);
		}
	fix.valid.speed ? gpsSpeedmps = fix.speed() * 0.514444 : 0;
	fix.satellites ? sats = fix.satellites : 0;
	if(fix.valid.heading){
		heading = fix.heading() * 0.01745329251;
		heading && headingOffst ? relHeading = RoundRadPosNeg(heading - headingOffst) : 0;
		relHeading ? relYawAngle = -relHeading : 0;
		for(int8_t i=0;i<2;i++){
			distToCP[i].t ? relHeadingTgt[i] = RoundRadPosNeg(heading-distToCP[i].t) : 0;
			relHeadingTgt[i] ? relYawAngleTgt[i] = -relHeadingTgt[i] : 0;
			}
		}
#ifdef DEBUG_GPS
	Serial.print("Sats:,"); Serial.print(sats);
	Serial.print(",Lat:,"); Serial.print(fix.latitude(),8); Serial.print(",Lon:,"); Serial.print(fix.longitude(),8);
	Serial.print(",PosN:,"); Serial.print(centerPos.r * cos(centerPos.t)); Serial.print(",PosE:,"); Serial.print(centerPos.r * sin(centerPos.t));
	Serial.print(",Heading:,"); Serial.print(heading); Serial.print(",relHeading:,");Serial.print(relHeading);
	Serial.print(",relYawAngle:,"); Serial.print(relYawAngle);Serial.print(",Speed:,"); Serial.println(gpsSpeedmps);
#endif
}

/************************************************************************
 * FUNCTION : IMU更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void IMUupdate(void)
{
	float gfx, gfy, gfz; //  Gyroscope raw values from MPU-6150
	float afx,afy,afz;
	int aix, aiy, aiz;
	int gix, giy, giz;
	float sampleTime;
	sampleTime = T10.getInterval() * 0.001;
	VectorFloat rpyAngleDeg;
	//FilterOnePole lowpassFilter(LOWPASS, 5.0);

	IMU.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

	afx = convertRawAcceleration(aiy);
	afy = -convertRawAcceleration(aix);
	afz = convertRawAcceleration(aiz);
	gfx = convertRawGyro(giy);
	gfy = -convertRawGyro(gix);
	gfz = convertRawGyro(giz);

	AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);

	rpyAngle.x ? rpyRate.x = CalcRPYRate(rpyAngle.x,AHRS.getRoll() * M_PI/180,sampleTime) : 0;
	rpyAngle.y ? rpyRate.y = CalcRPYRate(rpyAngle.y,AHRS.getPitch() * M_PI/180,sampleTime) : 0;
	rpyAngle.z ? rpyRate.z = CalcRPYRate(rpyAngle.z,AHRS.getYaw() * M_PI/180,sampleTime) : 0;

	rpyAngle.x = AHRS.getRoll() * M_PI/180; //ロール角
	rpyAngle.y = AHRS.getPitch() * M_PI/180; //ピッチ角
	rpyAngle.z = AHRS.getYaw() * M_PI/180; //ヨー角

	acc.x = convertRawAcceleration(aiy + (16384 * sin(rpyAngle.y) * cos(rpyAngle.x))); //重力の影響を除外した加速度x
	acc.y = convertRawAcceleration(-aix - (16384 * cos(rpyAngle.y) * sin(rpyAngle.x))); //重力の影響を除外した加速度y
	acc.z = convertRawAcceleration(aiz - (16384 * cos(rpyAngle.y) * cos(rpyAngle.x))); //重力の影響を除外した加速度z


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
	Serial.print("rollRt: ");
	Serial.print(rpyRate.x);
	Serial.print(" ");
	Serial.print("pitchRt: ");
	Serial.print(rpyRate.y);
	Serial.print(" ");
	Serial.print("yawRt: ");
	Serial.println(rpyRate.z);
#endif
}

float CalcRPYRate(float preAngle,float nowAngle,float sampleTime)
{
	float rate;
	rate = nowAngle - preAngle;
	if( rate < -3.14 ){
		rate += 6.28;
	}
	else if( rate > 3.14 ){
		rate -= 6.28;
	}
	rate /= sampleTime;
  return rate;
}

float LimitValue(float inputValue,float upperLimitValue,float lowerLimitValue)
{
	float buf;
	buf = min(inputValue,upperLimitValue);
	buf = max(buf,lowerLimitValue);
	return buf;
}

float Deadzone(float inputValue,float upperDeadzone,float lowerDeadzone,float center)
{
	float buf;
	if(inputValue>upperDeadzone){
		buf = inputValue;
	}
	else if(inputValue<lowerDeadzone){
		buf = inputValue;
	}
	else{
		buf = center;
	}
	return buf;
}

/************************************************************************
 * FUNCTION : IMU更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Initwait(void)
{
	// wait for ready
	while (Serial.available() && Serial.read()) ; // empty buffer
	Serial.println(F("Send any character to Start RobotCar!!: "));
	while (!Serial.available()) ;          // wait for data
	while (Serial.available() && Serial.read()) ; // empty buffer again
}

/************************************************************************
 * FUNCTION : コース設定
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void SetCourseData(float altitude, float geoid)
{

}
/************************************************************************
 * FUNCTION : コース中心からの相対位置計算
 * INPUT    : GPS緯度経度、標高、ジオイド高、コース中心位置(ECEF座標)、コースの角度(ECEF座標)
 * OUTPUT   : なし
 ***********************************************************************/
VectorFloat getRelPosition(polVectorFloat3D latlon, float alt, float geoid, VectorFloat center, float courseAngle)
{

}


/************************************************************************
 * FUNCTION : 現在位置推定
 * INPUT    : GPS緯度経度、標高、ジオイド高、コース中心位置(ECEF座標)、コースの角度(ECEF座標)
 * OUTPUT   : なし
 ***********************************************************************/
VectorFloat GetEstPosition(polVectorFloat3D latlon, float alt, float geoid, VectorFloat center, float courseAngle)
{

}

/************************************************************************
 * FUNCTION : サーボ出力(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void ChassisFinalOutput(int puPwm,int fStrPwm)
{
	PowUnit.write(puPwm);
	FStr.write(fStrPwm);
}


/************************************************************************
 * FUNCTION : シャシ統合制御(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void IntegratedChassisControl(void)
{
	#if 0
	static float targetAngleCP,targetYawRtCP;
	int8_t mode;
	VectorFloat targetpoint;
	!targetAngleCP ? targetAngleCP = atan2(clippingPoint2D[0].y - pos2D.y,clippingPoint2D[0].x - pos2D.x) : 0;
	mode = StateManager(pos2D,pointKeepOut,clippingPoint2D,relHeading,rpyRate);
	mode > 0 ? puPwm =  puPwm = 86 : BrakeCtrl(0,gpsSpeedmps,5);
	switch (mode) {
	case 1: targetpoint.x = clippingPoint2D[0].x+3;
					targetpoint.y = clippingPoint2D[0].y;
					targetYawRtCP = CalcTargetYawRt(mode,pos2D,targetpoint);
					fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate,targetYawRtCP);
					break;

	case 2: fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate,1.0f);
					break;

	case 3: targetYawRtCP = CalcTargetYawRt(mode,pos2D,clippingPoint2D[1]);
					fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate,targetYawRtCP);
					break;

	case 4: targetpoint.x = clippingPoint2D[1].x-3;
					targetpoint.y = clippingPoint2D[1].y;
					targetYawRtCP = CalcTargetYawRt(mode,pos2D,targetpoint);
					fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate,targetYawRtCP);
					break;

	case 5: fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate,-1.0f);
					break;

	case 6: targetYawRtCP = CalcTargetYawRt(mode,pos2D,clippingPoint2D[0]);
					fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate,targetYawRtCP);
					break;

	default:fStrPwm = 90;
					break;
	}
	#if 1
	#ifdef DEBUG
	Serial.print(",PosX:,"); Serial.print(pos2D.x); Serial.print(",PosY:,"); Serial.print(pos2D.y); Serial.print(",TgtAngle:,"); Serial.print(targetAngleCP);
	Serial.print(",TgtYawRt:,"); Serial.print(targetYawRtCP);Serial.print(",yawRate:,"); Serial.print(rpyRate.z,6); Serial.print(",Speed:,"); Serial.print(gpsSpeedmps);
	Serial.print(",Mode:,"); Serial.print(mode);Serial.print("StrPWM:,"); Serial.println(fStrPwm);
	#endif
	#endif
#endif
}


/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    :
 *
 * OUTPUT   : 状態値
 ***********************************************************************/
int8_t StateManager(NeoGPS::Location_t cp[],polVectorFloat3D distToCP[], polVectorFloat3D centerPos,float relYawAngleTgt[],float relHeading)
{
	static int8_t mode;
	float sampleTime;
	sampleTime = T10.getInterval() * 0.001;
	if(centerPos.r < 20) {			//コース中央から半径20m内
		if((mode != 1 || mode != 4) && cos(distToCP[0].p - headingOffst) > 0 && cos(distToCP[1].p + headingOffst) > 0) {
			cos(relHeading) > 0 ? mode = 1 : mode = 4;
		}
		if((mode != 2 || mode != 3) && cos(distToCP[0].p - headingOffst) < 0) {mode = 2;}
		if((mode != 5 || mode != 6) && cos(distToCP[1].p + headingOffst) < 0) {mode = 5;}
		if(mode == 2){
			sin(relHeading) > 0 && cos(relHeading) < 0 ? mode += 1 : 0;
		}
		if(mode == 5){
			sin(relHeading) > 0 && cos(relHeading) > 0 ? mode += 1 : 0;
		}
	}
	else{
		mode = -1;
	}
	//Serial.print(",Time:,"); Serial.print(lastProcessTime);
	return mode;

}

/************************************************************************
 * FUNCTION : 目標ヨーレート演算
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
float CalcTargetYawRt(int8_t mode, VectorFloat pos2D, VectorFloat targetclippingPoint2D){
#if 0
float TargetYawRt;
if(mode == 1 || mode == 6){
	TargetYawRt = 2 * (targetclippingPoint2D.y - pos2D.y) / (pow(targetclippingPoint2D.y - pos2D.y,2) + pow(targetclippingPoint2D.x - pos2D.x,2));
}
else{
	TargetYawRt = - 2 * (targetclippingPoint2D.y - pos2D.y) / (pow(targetclippingPoint2D.y - pos2D.y,2) + pow(targetclippingPoint2D.x - pos2D.x,2));
}
return TargetYawRt;
#endif
}


/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨーレートフィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControlPID(int8_t mode, float value, VectorFloat rpyRate,float targetYawRt)
{
	float sampleTime;
	float kp = 2.125,ti = 0.4 ,td = 0.1,diff;
	static float err,lastyawRate;
	static int8_t lastMode;
	if(mode != lastMode){
		err = 0;
		lastyawRate = 0;
	}
	!value ? value = 90 : 0;
	sampleTime = T10.getInterval() * 0.001;
	err  += (targetYawRt - rpyRate.z) * sampleTime;
	diff = 	(rpyRate.z - lastyawRate) / sampleTime;

  value +=  kp * (err/ti - (rpyRate.z + td * diff));

	value = LimitValue(value,120,60);

	lastyawRate = rpyRate.z;
	lastMode = mode;
#if 0
	#ifdef DEBUG
	Serial.print("Time,");Serial.print(millis());
	Serial.print(",err:,"); Serial.print(err);Serial.print(",diff:,"); Serial.print(diff); Serial.print(",TGTYawRt:,"); Serial.print(targetYawRt); Serial.print(",YawRt:,"); Serial.print(rpyRate.z);
	Serial.print(",value:,"); Serial.println(value);
	#endif
#endif

	return value;

}

/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨー角フィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControlPIDAng(float value,int8_t mode, VectorFloat rpyAngle,VectorFloat rpyRate,float targetAngleCP,float relHeading)
{
	float sampleTime;
	float kp = 5,ti = 0.4 ,td = 0.1;
	float relTA,diff;
	static float err[3], yawA,relTAOff,lastvalue;
	static int lastMode;

	!value ? value = 90 : 0;
	sampleTime = T10.getInterval() * 0.001;

	if(mode != lastMode) {
		yawA -= rpyAngle.z;
		relTAOff = relHeading;
	}
	yawA += rpyRate.z * sampleTime;
	relTA = RoundRad(targetAngleCP - relTAOff);
	err[0] = relTA - yawA;

	if(err[2] != 0){
	value = lastvalue + kp * ( err[0] - err[1] + sampleTime / ti * err[0] + td / sampleTime * (err[0] - 2 * err[1] + err[2]));
	}
	value = LimitValue(value,120,60);
	err[1] = err[0];
	err[2] = err[1];
	lastvalue = value;
	lastMode = mode;

#if 0
	#ifdef DEBUG
	Serial.print(",Mode:,"); Serial.print(mode);
	Serial.print(",err:,"); Serial.print(err[0]); Serial.print(",yawA:,"); Serial.print(yawA); Serial.print(",TgtAngle:,"); Serial.print(relTA);
	Serial.print(",yawAngle:,"); Serial.print(yawA); Serial.print(",value:,"); Serial.println(value);
	#endif
#endif
	return value;
}

/************************************************************************
 * FUNCTION : 速度コントロール
 * INPUT    :
 *
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float SpdControlPID(void)
{
}


int BrakeCtrl(float targetSpeed, float nowSpeedmps, float maxDecelAx)
{
	if(targetSpeed) {
		if(nowSpeedmps > targetSpeed) {
			acc.x > -maxDecelAx ? puPwm += 0.5 : puPwm -= 0.05;
		}
		else if(puPwm > 92) {        //バック防止
			puPwm = 92;
		}
	}
	else{                                //停止したい場合
		if(nowSpeedmps > 0.5) {
			acc.x > -maxDecelAx ? puPwm += 1.0 : puPwm -= 0.05;
		}
		else if(puPwm > 92) {      //バック防止
			puPwm = 92;
		}
	}
	puPwm = LimitValue(puPwm,180,0);
	return puPwm;
}


float convertRawAcceleration(int aRaw)
 {
	// since we are using 2G(19.6 m/s^2) range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32767

	float a = (aRaw * 2.0 * 9.80665) / 32768.0;
	return a;
}

float convertRawGyro(int gRaw)
 {
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

float RoundRadPosNeg(float x)//角度を表す変数を-3.14～3.14rad(M_PI)の範囲に収める関数
{
    return fmod(x + M_PI, 2 * M_PI) - M_PI;
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
void loop()
{
	runner.execute();
}

void serialEvent1(){
	while (gps.available(Serial1)) {
		GPSupdate();
	}
}
