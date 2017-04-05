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
//#define DEBUG_GPS
#define DEBUG
//#define TechCom											/*テストコース設定*/

#ifndef TechCom
#define HappiTow
#endif

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
double altitude,geoid;                                            /*GPS標高(m)、ジオイド高(m)*/
polVectorFloat3D gpsLatLon;                                       /*GPS緯度・経度(deg)*/
float puPwm = 92;                                                 /*パワーユニットのPWM*/
float fStrPwm = 90;                                               /*ステアリングのPWM*/
bool started;
uint32_t timer = millis();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

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
	GPSupdate();
	//GPSStrControl(0 ,90 * M_PI / 180, heading , 10  * M_PI / 180);
}

/************************************************************************
 * FUNCTION : 10 ms周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task20ms(void)
{
	IntegratedChassisControl();
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
	SetCourseData();
}

/************************************************************************
 * FUNCTION : GPS更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void GPSupdate(void)
{
	float sampleTime = 0.01f;
	static uint32_t lastProcessTime;

	lastProcessTime == 0 ? sampleTime = 0.01f : sampleTime = (millis() - lastProcessTime) * 0.001f;

	if(GPS.altitude.isUpdated() && GPS.altitude.isValid()) {
		altitude = GPS.altitude.meters();
	}
	if(GPS.geoid.isUpdated() && GPS.geoid.isValid()) {
		geoid = GPS.geoid.meters();
	}

	if(GPS.speed.isUpdated() && GPS.speed.isValid()) {
		gpsSpeedmps = GPS.speed.mps();
	}
	else{//GPS情報受信・更新できない間は縦加速度による速度補正
		gpsSpeedmps += acc.x * sampleTime;
	}
	if(gpsSpeedmps > 0.25f && GPS.course.isUpdated() && GPS.course.isValid()) {
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
	//GPSの緯度経度
	if(GPS.location.isUpdated() && GPS.location.isValid()) {
		gpsLatLon.t = GPS.location.lat();
		gpsLatLon.p = GPS.location.lng();
	}
	pos2D = GetEstPosition(gpsLatLon, altitude, geoid, center, courseAngle);
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

	lastProcessTime = millis();

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

	if(rpyAngle.calcRad2Deg().x) {rpyRate.x = LimitValue((AHRS.getRoll()-rpyAngle.calcRad2Deg().x)/sampleTime,1000.0f,-1000.0f) * M_PI/180;}
	if(rpyAngle.calcRad2Deg().y) {rpyRate.y = LimitValue((AHRS.getPitch()-rpyAngle.calcRad2Deg().y)/sampleTime,1000.0f,-1000.0f) * M_PI/180;}
	if(rpyAngle.calcRad2Deg().z) {rpyRate.z = LimitValue((AHRS.getYaw()-rpyAngle.calcRad2Deg().z)/sampleTime,1000.0f,-1000.0f) * M_PI/180;}

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
	lastProcessTime = millis();
}

float LimitValue(float inputValue,float upperLimitValue,float lowerLimitValue)
{
	float buf;
	buf = min(inputValue,upperLimitValue);
	buf = max(buf,lowerLimitValue);
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
void SetCourseData(void)
{

#ifdef TechCom
		/*********コースの座標を入力*********/
		clippingPoint[0].t = 36.57592010f;  clippingPoint[1].t = 36.57586669f;/*緯度設定*/
		clippingPoint[0].p = 140.01605224f;  clippingPoint[1].p = 140.01580810f;/*経度設定*/
		/*******立ち入り禁止エリア設定*******/
		pointKeepOut[0].x = -20;   pointKeepOut[1].x = 20;/*立ち入り禁止エリア設定x(前後)方向*/
		pointKeepOut[0].y = -20;   pointKeepOut[1].y = 20;/*立ち入り禁止エリア設定y(横)方向*/
		/********************************/
#elif defined HappiTow
	/*********コースの座標を入力*********/
		clippingPoint[0].t = 36.56811523f;  clippingPoint[1].t = 36.56797790f;    /*緯度設定*/
		clippingPoint[0].p = 139.99578857f;  clippingPoint[1].p = 139.99578857f;  /*経度設定*/
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

		Serial.print("CPbuf0:"); Serial.print(buf0[0].x); Serial.print(","); Serial.println(buf0[0].y);
		Serial.print("CPbuf1:"); Serial.print(buf0[1].x); Serial.print(","); Serial.println(buf0[1].y);

		for(uint8_t i=0; i<2; i++) {
			clippingPoint2D[i].x = buf0[i].x * cos(-courseAngle) - buf0[i].y * sin(-courseAngle);
			clippingPoint2D[i].y = buf0[i].x * sin(-courseAngle) + buf0[i].y * cos(-courseAngle);
		}

		Serial.print("CP0:"); Serial.print(clippingPoint2D[0].x); Serial.print(","); Serial.println(clippingPoint2D[0].y);
		Serial.print("CP1:"); Serial.print(clippingPoint2D[1].x); Serial.print(","); Serial.println(clippingPoint2D[1].y);
		Serial.print("CenterX:"); Serial.print(center.x);
		Serial.print("CenterY:"); Serial.println(center.y);


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
 * FUNCTION : シャシ統合制御(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void IntegratedChassisControl(void)
{
	int posModex;
	static float targetAngleCP,turnAngle;
	float sampleTime = 0.01f;
	static uint32_t lastProcessTime;
	lastProcessTime == 0 ? sampleTime = 0.02f : sampleTime = (millis() - lastProcessTime) * 0.001f;

	if(pointKeepOut[0].x < pos2D.x && pointKeepOut[1].x > pos2D.x && pointKeepOut[0].y < pos2D.y && pointKeepOut[1].y > pos2D.y) {
		PowUnit.write(87);
	}
	else{
		PowUnit.write(BrakeCtrl(0,gpsSpeedmps,5));
	}
	if(clippingPoint2D[0].x < pos2D.x && abs(turnAngle) < 3.14) {
		if(cos(relAngle) < 0) {
			targetAngleCP = -atan2((clippingPoint2D[1].y - pos2D.y),(clippingPoint2D[1].x - pos2D.x));
			if(sin(relAngle) < 0){posModex = 0;}
		}
		else{posModex = 1;}
		turnAngle += rpyRate.z * sampleTime;
	}
	else if(pos2D.x < clippingPoint2D[1].x && abs(turnAngle) < 3.14) {
		if(cos(relAngle) > 0) {
			targetAngleCP = -atan2((clippingPoint2D[0].y - pos2D.y),(clippingPoint2D[0].x - pos2D.x));
			if(sin(relAngle) < 0){posModex = 0;}
		}
		else{posModex = -1;}
		turnAngle += rpyRate.z * sampleTime;
	}
	else{
		posModex = 0;
		turnAngle = 0;
	}
    #ifdef DEBUG
	Serial.print("Lat:,"); Serial.print(gpsLatLon.t,8); Serial.print(",Lon:,"); Serial.print(gpsLatLon.p,8);
	Serial.print(",PosX:,"); Serial.print(pos2D.x); Serial.print(",PosY:,"); Serial.print(pos2D.y);
	Serial.print(",Heading:,"); Serial.print(heading); Serial.print(",RelAngle:,"); Serial.print(relAngle); Serial.print(",Speed:,"); Serial.println(gpsSpeedmps);
	Serial.print("Mode:,"); Serial.print(posModex); Serial.print("TargetAngle:,"); Serial.println(targetAngleCP);
    #endif
	switch (posModex) {
	case 0: FStr.write((int)StrControl(targetAngleCP,rpyRate,posModex)); break;
	case 1: FStr.write((int)StrControl(targetAngleCP,rpyRate,posModex)); break;
	case -1: FStr.write((int)StrControl(targetAngleCP,rpyRate,posModex)); break;
	default: BrakeCtrl(0,gpsSpeedmps,5); break;
	}
	lastProcessTime = millis();
}

/************************************************************************
 * FUNCTION : 操舵制御指示値演算
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControl(float targetAngleCP,VectorFloat rpyRate,int forceCtrlMode)
{
	float strSpeedGain = 2;         //操舵速度ゲイン
	float thresholdAngleRad = 0.3;
	static float controlValue = 90.0f; //直進状態を初期値とする
	strSpeedGain += abs(rpyRate.z);

	if(forceCtrlMode == 0){
		 	if (sin(targetAngleCP - relAngle) < sin(-thresholdAngleRad)) {//右にずれてる
			 	controlValue += strSpeedGain * abs(sin(targetAngleCP - relAngle)); //左にずれ分修正
			}
			else if(sin(thresholdAngleRad) < sin(targetAngleCP - relAngle)) {//左にずれてる
				controlValue -= strSpeedGain * abs(sin(targetAngleCP - relAngle));//右にずれ分修正
			}
			else{
				controlValue = StrControlStraight(controlValue,targetAngleCP,rpyRate,strSpeedGain);
			}
		}
	if(forceCtrlMode == 1){
		if(abs(sin(targetAngleCP - relAngle)) > sin(thresholdAngleRad)) {
			controlValue += strSpeedGain * abs(sin(targetAngleCP - relAngle)); //左旋回でずれ分修正
		}
		else{
			controlValue = StrControlStraight(controlValue,targetAngleCP,rpyRate,strSpeedGain);
		}
	}
	if(forceCtrlMode == -1){
		if(abs(sin(targetAngleCP - relAngle)) > sin(thresholdAngleRad)) {
			controlValue -= strSpeedGain * abs(sin(targetAngleCP - relAngle)); //右旋回でずれ分修正
		}
		else{
			controlValue = StrControlStraight(controlValue,targetAngleCP,rpyRate,strSpeedGain);
		}
	}
	controlValue = LimitValue(controlValue,120,60);
#ifdef DEBUG
	if(controlValue > 90) {Serial.println("Turn L");}
	else if(controlValue < 90) {Serial.println("Turn R");}
	else{Serial.println("No Steer");}
#endif
	return controlValue;
}

float StrControlStraight(float controlValue,float targetAngleCP,VectorFloat rpyRate,float strSpeedGain)
{ controlValue > 90 ? controlValue -= 0.1 : 0;
	controlValue < 90 ? controlValue += 0.1 : 0;
	controlValue == 90 ? controlValue -= strSpeedGain * rpyRate.z : 0;
	controlValue = LimitValue(controlValue,120,60);
	return controlValue;
}







/*GPS方位ベース操舵処理(方位判定まで)*/
void GPSStrControl(int directionMode, float tgtAngleRad, float nowAngleRad, float thresholdAngleRad)  //コース走行時のestAngleはクリッピングポイントから演算して入力する
{
	float limCircleMin[2],limCircleMax[2];
	float diffAngleRad;
	bool isGoStraight;

	//目標方位と現在方位差を円として表現
	//目標方位と現在方位差の許容範囲を決める(thresholdAngleRad)
	limCircleMin[0] = 1-cos(thresholdAngleRad); //円x方向最小値
	limCircleMin[1] = sin(-thresholdAngleRad); //円y方向最小値
	limCircleMax[0] = 1;                  //円x方向最大値(演算では使用せず)
	limCircleMax[1] = sin(thresholdAngleRad); //円y方向最大値

	//目標方位と現在方位の差
	diffAngleRad = tgtAngleRad - nowAngleRad;
#ifdef DEBUG
	Serial.print(nowAngleRad);
	Serial.print("TurnDirection:");
#endif
	//GPS方位による直進判定
	isGoStraight = (limCircleMin[0] < cos(diffAngleRad)) &&
	               (limCircleMin[1] < sin(diffAngleRad)) &&
	               (sin(diffAngleRad) < limCircleMax[1]);

//switch(directionMode){                        //旋回方向モード(-1:右,1:左,それ以外:左右)
	//case -1:
	//StrCtrlR(isGoStraight);break;
//  case 1:
	//StrCtrlL(isGoStraight);break;
//  default:
	//StrCtrlLR(isGoStraight,diffAngleRad);break;
}

uint8_t ConstTurn(bool isGoStraight, int8_t direction, float turnRadius, float targetAngle, float maxAy)
{
	float estAy = gpsSpeedmps*gpsSpeedmps/turnRadius;
	float estYawRt = maxAy/gpsSpeedmps;

	/*加減速制御*/
	/*予想旋回G,実測横GがmaxAy以上の場合は速度を下げる*/
	if(estAy > maxAy || abs(acc.y) > maxAy) {
		puPwm += 0.5;
	}
	else if(puPwm > 90) { /*バック防止*/
		puPwm -= 0.1;
	}

	/*旋回制御*/
	if(!isGoStraight) {
		if(estAy < maxAy || abs(acc.y) < maxAy || estYawRt < abs(rpyRate.z)) {
			fStrPwm += (float)direction * 0.5;
		}
		else if(estYawRt > abs(rpyRate.z)) {
			fStrPwm -= (float)direction * 0.2; /*ヨーレートが過剰な場合はカウンタを当てる*/
		}
	}
	else{
		puPwm = 90;
	}
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
	volatile uint8_t cnt2,cnt10,cnt100;
	if (timer > millis()) timer = millis();
	if (millis() - timer >= 10) {
		Task10ms();
		if(cnt2>=2) {
			Task20ms();
			cnt2=0;
		}
		if(cnt10>=10) {
			Task100ms();
			cnt10=0;
		}
		if(cnt100>=100) {
			Task1000ms();
			cnt100=0;
		}
		timer = millis();
		cnt2++;
		cnt10++;
		cnt100++;
	}
}

void loop()
{
	TaskMain();
}

void serialEvent1(){
	while (Serial1.available() > 0) {
		GPS.encode(Serial1.read());
	}
}
