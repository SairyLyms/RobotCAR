/************************************************************************/
/*	Include Files														*/
/************************************************************************/
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

/************************************************************************/
/*  DEFINE Earth values                   */
/************************************************************************/
#define A 6378137.0
/* Semi-major axis */
#define ONE_F 298.257223563
/* 1/F */
#define E2  ((1.0/ONE_F)*(2-(1.0/ONE_F)))
#define NN(p) (A/sqrt(1.0 - (E2)*pow(sin(p*M_PI/180.0),2)))
// ================================================================
// ===                       Course Data                        ===
// ================================================================
VectorFloat center;
polVectorFloat3D clippingPoint[2];
VectorFloat pointKeepOut[2];        /*立ち入り禁止エリア設定*/

//#define DEBUG_IMU
#define DEBUG_GPS
//#define DEBUG
//#define TechCom											/*テストコース設定*/

#ifndef TechCom
//#define HappiTow
#define Garden
#endif

TinyGPSPlus GPS;
MPU6050 IMU;
HMC5883L MAG;
Madgwick AHRS;
Servo FStr,PowUnit;
Scheduler runner;
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
double altitude,geoid;                                            /*GPS標高(m)、ジオイド高(m)*/
polVectorFloat3D gpsLatLon;                                       /*GPS緯度・経度(deg)*/
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
	SetCourseData(altitude,geoid);
}

/************************************************************************
 * FUNCTION : GPS更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void GPSupdate(void)
{
	if(GPS.altitude.isUpdated() && GPS.altitude.isValid() && GPS.altitude.meters()) {
		altitude = (float)GPS.altitude.meters();
	}
	if(GPS.geoid.isUpdated() && GPS.geoid.isValid() && GPS.geoid.meters()) {
		geoid = (float)GPS.geoid.meters();
	}
	if(GPS.speed.isUpdated() && GPS.speed.isValid() && GPS.speed.mps()) {
		gpsSpeedmps = (float)GPS.speed.mps();
	}
	if(GPS.course.isUpdated() && GPS.course.isValid() && GPS.course.rad()) {
		heading = -(float)GPS.course.rad();                     //headingは左旋回方向を正にする
		headingDeg = heading * 180 / M_PI;
	}
	//GPSの緯度経度
	if(GPS.location.isUpdated() && GPS.location.isValid()) {
		if(GPS.location.lat()) {gpsLatLon.t = (float)GPS.location.lat();}
		if(GPS.location.lng()) {gpsLatLon.p = (float)GPS.location.lng();}
	}
	pos2D = getRelPosition(gpsLatLon, altitude, geoid, center, courseAngle);
	relAngle = RoundRad(heading - courseAngle);
#ifdef DEBUG_GPS
	Serial.print("Lat:,"); Serial.print(gpsLatLon.t,8); Serial.print(",Lon:,"); Serial.print(gpsLatLon.p,8);
	Serial.print(",Alt:,"); Serial.print(altitude); Serial.print(",Geo:,"); Serial.print(geoid);
	Serial.print(",PosX:,"); Serial.print(pos2D.x); Serial.print(",PosY:,"); Serial.print(pos2D.y);
	Serial.print(",Heading:,"); Serial.print(heading); Serial.print(",RelAngle:,"); Serial.print(relAngle); Serial.print(",Speed:,"); Serial.println(gpsSpeedmps);
#endif
	//Serial.print(blh2ecefx(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
	//Serial.print(",");
	//Serial.print(blh2ecefy(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
	//Serial.print(",");
	//Serial.println(blh2ecefz(gpsLatDeg, gpsLonDeg,GPS.altitude.meters() , GPS.geoid.meters()));
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

	if(rpyAngle.calcRad2Deg().x) {rpyRate.x = (LimitValue((AHRS.getRoll()-rpyAngle.calcRad2Deg().x)/sampleTime,1000.0f,-1000.0f) * M_PI/180);}
	if(rpyAngle.calcRad2Deg().y) {rpyRate.y = (LimitValue((AHRS.getPitch()-rpyAngle.calcRad2Deg().y)/sampleTime,1000.0f,-1000.0f) * M_PI/180);}
	if(rpyAngle.calcRad2Deg().z) {rpyRate.z = (LimitValue((AHRS.getYaw()-rpyAngle.calcRad2Deg().z)/sampleTime,1000.0f,-1000.0f) * M_PI/180);}

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
	Serial.print("roll: ");
	Serial.print(rpyRate.x);
	Serial.print(" ");
	Serial.print("pitch: ");
	Serial.print(rpyRate.y);
	Serial.print(" ");
	Serial.print("yaw: ");
	Serial.println(rpyRate.z);
#endif
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
	Serial.println(F("Send any character to Start RobotCar!!: "));
	while (Serial.available() && Serial.read()) ; // empty buffer
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
	if(altitude && geoid) {

#ifdef TechCom
		/*********コースの座標を入力*********/
		clippingPoint[0].t = 36.56809997f;  clippingPoint[1].t = 36.57586669f;/*緯度設定*/
		clippingPoint[0].p = 139.99588012f;  clippingPoint[1].p = 140.01580810f;/*経度設定*/
		/*******立ち入り禁止エリア設定*******/
		pointKeepOut[0].x = -20;   pointKeepOut[1].x = 20;/*立ち入り禁止エリア設定x(前後)方向*/
		pointKeepOut[0].y = -20;   pointKeepOut[1].y = 20;/*立ち入り禁止エリア設定y(横)方向*/
		/********************************/
#elif defined HappiTow
		/*********コースの座標を入力*********/
		clippingPoint[0].t = 36.56809997;  clippingPoint[1].t = 36.56797027;    /*緯度設定*/
		clippingPoint[0].p = 139.99595642f;  clippingPoint[1].p = 139.99595642f;  /*経度設定*/
		/*******立ち入り禁止エリア設定*******/
		pointKeepOut[0].x = -20;   pointKeepOut[1].x = 20;/*立ち入り禁止エリア設定x(前後)方向*/
		pointKeepOut[0].y = -20;   pointKeepOut[1].y = 20;/*立ち入り禁止エリア設定y(横)方向*/
		/********************************/
#elif defined Garden
				/*********コースの座標を入力*********/
				clippingPoint[0].t = 36.57806396f;  clippingPoint[1].t = 36.57814788f;    /*緯度設定*/
				clippingPoint[0].p = 140.01487731f;  clippingPoint[1].p = 140.01506042f;  /*経度設定*/
				/*******立ち入り禁止エリア設定*******/
				pointKeepOut[0].x = -20;   pointKeepOut[1].x = 20;/*立ち入り禁止エリア設定x(前後)方向*/
				pointKeepOut[0].y = -20;   pointKeepOut[1].y = 20;/*立ち入り禁止エリア設定y(横)方向*/
				/********************************/
#endif

		VectorFloat buf0[2];

		for(uint8_t i=0; i<2; i++) {
			buf0[i] = blh2ecef(clippingPoint[i],altitude,geoid);
		}

		center.x = 0.5*(buf0[1].x+buf0[0].x);
		center.y = 0.5*(buf0[1].y+buf0[0].y);

		for(uint8_t i=0; i<2; i++) {
			buf0[i].x = buf0[i].x - center.x;
			buf0[i].y = buf0[i].y - center.y;
		}

		courseAngle = atan2(buf0[0].y,buf0[0].x);

		for(uint8_t i=0; i<2; i++) {
			clippingPoint2D[i].x = buf0[i].x * cos(-courseAngle) - buf0[i].y * sin(-courseAngle);
			clippingPoint2D[i].y = buf0[i].x * sin(-courseAngle) + buf0[i].y * cos(-courseAngle);
		}

		Serial.print("CP0:"); Serial.print(clippingPoint2D[0].x); Serial.print(","); Serial.println(clippingPoint2D[0].y);
		Serial.print("CP1:"); Serial.print(clippingPoint2D[1].x); Serial.print(","); Serial.println(clippingPoint2D[1].y);
		Serial.print("CenterX:"); Serial.print(center.x);
		Serial.print("CenterY:"); Serial.println(center.y);

	}

}

/************************************************************************
 * FUNCTION : コース中心からの相対位置計算
 * INPUT    : GPS緯度経度、標高、ジオイド高、コース中心位置(ECEF座標)、コースの角度(ECEF座標)
 * OUTPUT   : なし
 ***********************************************************************/
VectorFloat getRelPosition(polVectorFloat3D latlon, float alt, float geoid, VectorFloat center, float courseAngle)
{
	VectorFloat buf0,relPos2D;
	buf0 = blh2ecef(latlon,alt,geoid);

	buf0.x = buf0.x - center.x;
	buf0.y = buf0.y - center.y;

	relPos2D.x = buf0.x * cos(-courseAngle) - buf0.y * sin(-courseAngle);
	relPos2D.y = buf0.x * sin(-courseAngle) + buf0.y * cos(-courseAngle);

	return relPos2D;
}


/************************************************************************
 * FUNCTION : 現在位置推定
 * INPUT    : GPS緯度経度、標高、ジオイド高、コース中心位置(ECEF座標)、コースの角度(ECEF座標)
 * OUTPUT   : なし
 ***********************************************************************/
VectorFloat GetEstPosition(polVectorFloat3D latlon, float alt, float geoid, VectorFloat center, float courseAngle)
{
	float sampleTime = 0.01f;
	static uint32_t lastProcessTime;
	static float deltaAngle;                //GPS更新する間に変化した角度(rad)
	static polVectorFloat3D lastLatLon;     //最後に更新したGPSデータ
	static VectorFloat deltaPos2D;          //GPS更新する間に移動した距離(X,Y)
	static VectorFloat updatedPos2D;        //出力用の距離データ(X,Y)
	VectorFloat estPos2D,deltaGPSpos2D;

	lastProcessTime == 0 ? sampleTime = 0.01f : sampleTime = (millis() - lastProcessTime) * 0.001f;

	if(lastLatLon.t != latlon.t || lastLatLon.p != latlon.p) {
		estPos2D.x = getRelPosition(lastLatLon,alt,geoid,center,courseAngle).x + deltaPos2D.x;
		estPos2D.y = getRelPosition(lastLatLon,alt,geoid,center,courseAngle).y + deltaPos2D.y;
		deltaGPSpos2D.x = getRelPosition(latlon,alt,geoid,center,courseAngle).x - getRelPosition(lastLatLon,alt,geoid,center,courseAngle).x;
		deltaGPSpos2D.y = getRelPosition(latlon,alt,geoid,center,courseAngle).y - getRelPosition(lastLatLon,alt,geoid,center,courseAngle).y;
		//両者の距離の大きさの差が2m以下の場合
		if(abs(getRelPosition(latlon,alt,geoid,center,courseAngle).getMagnitude()-estPos2D.getMagnitude()) < 2) {
			float deltaGPSAngle = atan2(deltaGPSpos2D.y,deltaGPSpos2D.x);
			if(abs(deltaAngle) - abs(deltaGPSAngle) > 0.5) {
				//方位の差が0.5rad超の場合、推定値で更新
				updatedPos2D = estPos2D;
				//Serial.println("estimate_u2");
			}
			else{
				//方位の差が0.5rad以下の場合、GPSデータで更新
				updatedPos2D = getRelPosition(latlon,alt,geoid,center,courseAngle);
				//Serial.println("GPS_u2");
			}
		}
		else{
			//2m超の差があれば有意な差とみなし、GPSの緯度経度の更新値が進行方向前方±1rad以下であったらGPSデータで更新
			if(abs(atan2(deltaGPSpos2D.y,deltaGPSpos2D.x)) <= 1) {
				updatedPos2D.x = getRelPosition(latlon,alt,geoid,center,courseAngle).x;
				updatedPos2D.y = getRelPosition(latlon,alt,geoid,center,courseAngle).y;
				//Serial.println("GPS_o2");
			}
			else{//2m超の差があったが、GPSの緯度経度の更新値が進行方向±1radを超えていた場合は推定値で更新
				updatedPos2D = estPos2D;
				//Serial.println("estimate_o2");
			}
		}
		deltaAngle = 0;
		deltaPos2D.x = 0;
		deltaPos2D.y = 0;
		lastLatLon = latlon;
	}
	else{
		updatedPos2D.x += gpsSpeedmps * cos(relAngle) * sampleTime;
		updatedPos2D.y += gpsSpeedmps * sin(relAngle) * sampleTime;
	}
	deltaAngle -= rpyRate.z * sampleTime;
	deltaPos2D.x += gpsSpeedmps * cos(relAngle) * sampleTime;
	deltaPos2D.y += gpsSpeedmps * sin(relAngle) * sampleTime;
	lastProcessTime = millis();
	return updatedPos2D;
}

/************************************************************************
 * FUNCTION : BLH座標系からECEF座標系への変換
 * INPUT    : GPS緯度経度、標高、ジオイド高
 * OUTPUT   : なし
 ***********************************************************************/
VectorFloat blh2ecef(polVectorFloat3D LatLon, float alt, float geoid)
{
	VectorFloat ecef;
	ecef.x = (NN(LatLon.t)+(alt+geoid))*cos(LatLon.t*M_PI/180)*cos(LatLon.p*M_PI/180);
	ecef.y = (NN(LatLon.t)+(alt+geoid))*cos(LatLon.t*M_PI/180)*sin(LatLon.p*M_PI/180);
	ecef.z = (NN(LatLon.t)*(1-E2)+(alt+geoid))*sin(LatLon.t*M_PI/180);
	return ecef;
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
	static float targetAngleCP,targetYawRtCP;
	int8_t mode;
	VectorFloat targetpoint;
	!targetAngleCP ? targetAngleCP = atan2(clippingPoint2D[0].y - pos2D.y,clippingPoint2D[0].x - pos2D.x) : 0;
	mode = StateManager(pos2D,pointKeepOut,clippingPoint2D,relAngle,rpyRate);
	mode > 0 ? puPwm =  puPwm = 87 : BrakeCtrl(0,gpsSpeedmps,5);
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
}


/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    :
 *
 * OUTPUT   : 状態値
 ***********************************************************************/
int8_t StateManager(VectorFloat pos2D, VectorFloat pointKeepOut[], VectorFloat clippingPoint2D[],float relAngle,VectorFloat rpyRate)
{
	static int8_t mode;
	float sampleTime;
	static float headingOffset;

	sampleTime = T10.getInterval() * 0.001;

	if(pointKeepOut[0].x < pos2D.x && pointKeepOut[1].x > pos2D.x && pointKeepOut[0].y < pos2D.y && pointKeepOut[1].y > pos2D.y) {
		if(clippingPoint2D[1].x < pos2D.x && pos2D.x < clippingPoint2D[0].x) {
			cos(relAngle) > 0 ? mode = 1 : mode = 4;
			if( -1 < pos2D.x && pos2D.x < 1 ) {headingOffset = 0;}
		}
		if((mode != 2 || mode != 3) && clippingPoint2D[0].x < pos2D.x) {mode = 2; headingOffset = heading;}
		if((mode != 5 || mode != 6) && pos2D.x < clippingPoint2D[1].x) {mode = 5; headingOffset = heading;}
		if(mode == 2 || mode == 5){
		cos(heading - headingOffset) < -0.8 ? mode += 1 : 0;
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
float TargetYawRt;

if(mode == 1 || mode == 6){
	TargetYawRt = 2 * (targetclippingPoint2D.y - pos2D.y) / (pow(targetclippingPoint2D.y - pos2D.y,2) + pow(targetclippingPoint2D.x - pos2D.x,2));
}
else{
	TargetYawRt = - 2 * (targetclippingPoint2D.y - pos2D.y) / (pow(targetclippingPoint2D.y - pos2D.y,2) + pow(targetclippingPoint2D.x - pos2D.x,2));
}
return TargetYawRt;
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
float StrControlPIDAng(float value,int8_t mode, VectorFloat rpyAngle,VectorFloat rpyRate,float targetAngleCP,float relAngle)
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
		relTAOff = relAngle;
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
	while (Serial1.available() > 0) {
		if(GPS.encode(Serial1.read())) {
			GPSupdate();
			break;
		}
	}
}
