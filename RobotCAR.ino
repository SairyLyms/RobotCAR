/************************************************************************/
/*	Include Files														*/
/************************************************************************/
#include <NMEAGPS.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
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
polVectorFloat3D distToCP[2];

#define CC01
//#define DEBUG_IMU
//#define DEBUG_GPS
//#define DEBUG
//#define TechCom											/*テストコース設定*/
//#define Ground
//#define Garden
#define HappiTow

#ifdef TechCom
NeoGPS::Location_t cp1(365759506L,1400158539L);
NeoGPS::Location_t cp2(365760193L,1400160370L);
#elif defined Garden
NeoGPS::Location_t cp1(365780240L,1400148780L);
NeoGPS::Location_t cp2(365780920L,1400151410L);
#elif defined Ground
NeoGPS::Location_t cp1(365833090L,1400104240L);
NeoGPS::Location_t cp2(365832140L,1400104870L);
#elif defined HappiTow
NeoGPS::Location_t cp1(365679436L,1399957886L);
NeoGPS::Location_t cp2(365680389L,1399957886L);
#endif

NeoGPS::Location_t cp[2] = {cp1,cp2};
NeoGPS::Location_t locCenter((long)(0.5*(cp[0].lat()+cp[1].lat())),(long)(5*(0.1 * cp[0].lon()+ 0.1 * cp[1].lon())));
NeoGPS::Location_t cpAway[2] = {cp1,cp2};

NeoGPS::Location_t relPos2D;
static NMEAGPS  gps;
static gps_fix  fix;
MPU6050 IMU;
Madgwick AHRS;
Servo FStr,PowUnit;
Scheduler runner;
/*座標系は安部先生の本に従う(右手系)*/
VectorFloat rpyAngle;
VectorFloat rpyRate;
VectorFloat acc;                                                  /*X,Y,Z加速度(m/s^2)*/
float relHeading;							 																		/*CP間中心線からの相対角度(Heading:CW+,Yaw:CCW+)*/
float relHeadingTgt[2];																						/*CPまでの角度(Heading:CW+)*/
float heading, headingDeg;																				/*方位(rad,deg)*/
float headingOffst = cp[1].BearingTo(cp[0]);											/*CP間の方位*/
float headingOffstIMU;
float gpsSpeedmps;                                               /*GPS絶対速度(m/s)*/
polVectorFloat3D centerPos;
uint8_t sats;
int puPwm = 90;																									/*パワーユニットのPWM*/
int fStrPwm = 90;                                               /*ステアリングのPWM*/

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

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
	Wire.begin();
	Serial.begin(115200);
	Serial1.begin(38400);
	//IMU.setI2CMasterModeEnabled(false);
	//IMU.setI2CBypassEnabled(true);
	//IMU.setSleepEnabled(false);
	IMU.initialize();
	IMU.setXAccelOffset(362);
	IMU.setYAccelOffset(-3714);
	IMU.setZAccelOffset(628);
	IMU.setXGyroOffset(109);
	IMU.setYGyroOffset(-16);
	IMU.setZGyroOffset(11);
	IMU.setDLPFMode(MPU6050_DLPF_BW_10);
	FStr.attach(3);
	PowUnit.attach(2,1000,2000);
	Serial.flush();
	runner.startNow();
	cpAway[0].OffsetBy( 0.1 / NeoGPS::Location_t::EARTH_RADIUS_KM, cp[1].BearingTo(cp[0]));
	cpAway[1].OffsetBy( 0.1 / NeoGPS::Location_t::EARTH_RADIUS_KM, cp[0].BearingTo(cp[1]));
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
	static unsigned long lastTime;
	float sampleTime = 0.1;
	fix = gps.read();
	lastTime ? sampleTime = (millis() - lastTime)*0.001 : 0;
	lastTime = millis();
	if(fix.valid.location) {
		for(int8_t i=0;i<2;i++){
			distToCP[i].r = fix.location.DistanceKm(cp[i]) * 1000.0f;
			distToCP[i].t = fix.location.BearingTo(cpAway[i]); //直進時制御用のリファレンス方位
			distToCP[i].p = fix.location.BearingTo(cp[i]); //旋回開始判定用の方位
			}
			centerPos.r = fix.location.DistanceKm(locCenter) * 1000.0f;
			centerPos.t = fix.location.BearingTo(locCenter);
		}
	fix.valid.speed ? gpsSpeedmps = fix.speed() * 0.514444 : 0;
	fix.valid.satellites ? sats = fix.satellites : 0;
	if(fix.valid.heading){
		heading = fix.heading() * 0.01745329251;
		heading && headingOffst ? relHeading = RoundRadPosNeg(heading - headingOffst) : 0;
		}
#ifdef DEBUG_GPS
	Serial.print("Sats:,"); Serial.print(sats);
	Serial.print(",Lat:,"); Serial.print(fix.latitude(),8); Serial.print(",Lon:,"); Serial.print(fix.longitude(),8);
	Serial.print(",PosN:,"); Serial.print(centerPos.r * cos(centerPos.t)); Serial.print(",PosE:,"); Serial.print(centerPos.r * sin(centerPos.t));
	Serial.print(",Heading:,"); Serial.print(heading); Serial.print(",relHeading:,");Serial.print(relHeading);
	Serial.print(",YawRtGPS:,"); Serial.print(yawRtGPS);Serial.print(",Speed:,"); Serial.println(gpsSpeedmps);
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

	IMU.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

	afx = convertRawAcceleration(aiy);
	afy = -convertRawAcceleration(aix);
	afz = convertRawAcceleration(aiz);

	gfx = convertRawGyro(giy);
	gfy = -convertRawGyro(gix);
	gfz = convertRawGyro(giz);

	AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);

	rpyRate.x = gfx * M_PI/180;
	rpyRate.y = gfy * M_PI/180;
	rpyRate.z = gfz * M_PI/180;

	rpyAngle.x = AHRS.getRoll() * M_PI/180; //ロール角
	rpyAngle.y = AHRS.getPitch() * M_PI/180; //ピッチ角
	rpyAngle.z = AHRS.getYaw() * M_PI/180; //ヨー角

	acc.x = convertRawAcceleration(aiy + (16384 * sin(rpyAngle.y) * cos(rpyAngle.x))); //重力の影響を除外した加速度x
	acc.y = convertRawAcceleration(-aix - (16384 * cos(rpyAngle.y) * sin(rpyAngle.x))); //重力の影響を除外した加速度y
	acc.z = convertRawAcceleration(aiz - (16384 * cos(rpyAngle.y) * cos(rpyAngle.x))); //重力の影響を除外した加速度z


#ifdef DEBUG_IMU
	Serial.print("Time, ");
	Serial.print(",");
	Serial.print(millis() * 0.001);
	Serial.print("Ax, ");
	Serial.print(acc.x);
	Serial.print(",");
	Serial.print("Ay, ");
	Serial.print(acc.y);
	Serial.print(",");
	Serial.print("Az, ");
	Serial.print(acc.z);
	Serial.print(",");
	Serial.print("rollRt, ");
	Serial.print(rpyRate.x);
	Serial.print(",");
	Serial.print("pitchRt, ");
	Serial.print(rpyRate.y);
	Serial.print(",");
	Serial.print("yawRt, ");
	Serial.print(rpyRate.z);
	Serial.print("yawAng, ");
	Serial.println(rpyAngle.z);
#endif
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
 * FUNCTION : サーボ出力(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void ChassisFinalOutput(int puPwm,int fStrPwm)
{
	PowUnit.write(puPwm);
#ifdef CC01
	FStr.write(180-fStrPwm);
#else
	FStr.write(fStrPwm);
#endif
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
	mode = StateManager(cp,distToCP,centerPos,relHeading);
	//switch (mode){
	switch (mode) {
	//0:デバッグ用
	case 0:
					puPwm = 90;
					break;
	//1:ヨー角キャリブレーション用
	case 1:
					puPwm = 110;
					fStrPwm = StrControlPID(mode,fStrPwm,rpyRate.z,0);
					break;

	case 2:	puPwm = 110;
					fStrPwm = StrControlPIDAng(mode,fStrPwm,RoundRadPosNeg(rpyAngle.z-headingOffstIMU+headingOffst),0);
					break;

	case 3: puPwm = 90;
					fStrPwm = StrControlPID(mode,fStrPwm,rpyRate.z,1);
					break;

	case 4: puPwm = 90;
					break;

	case 5: puPwm = 90;
					break;

	case 6: puPwm = 90;
					break;

	case 7: puPwm = 90;
					break;

	default:puPwm = 90;
					fStrPwm = (int)StrControlPID(mode,fStrPwm,rpyRate.z,0);											/*コース外ではγ-FBのデモを行う*/
					break;
	}
	Serial.print(",RoundRadPosNeg");Serial.println(RoundRadPosNeg(rpyAngle.z-headingOffstIMU + headingOffst));
	#if 0
	#ifdef DEBUG
	//Serial.print(",TgtAngle:,"); Serial.print(targetAngleCP);
	//Serial.print(",TgtYawRt:,"); Serial.print(targetYawRtCP);Serial.print(",yawRate:,"); Serial.print(rpyRate.z,6); Serial.print(",Speed:,"); Serial.print(gpsSpeedmps);
	//Serial.print(",Mode:,"); Serial.print(mode);Serial.print("StrPWM:,"); Serial.println(fStrPwm);
	#endif
	#endif
}


/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    :
 *
 * OUTPUT   : 状態値
 ***********************************************************************/
int8_t StateManager(NeoGPS::Location_t cp[],polVectorFloat3D distToCP[], polVectorFloat3D centerPos,float relHeading)
{
	static int8_t mode;
	static uint16_t initCount;
	static float buf = 0;
	if(sats && centerPos.r < 30) {			//コース中央から半径30m内
		if(initCount < 500){
			mode = 1;
			if(initCount == 499){
				headingOffstIMU = heading;
				Serial.print("Done:");Serial.println(headingOffstIMU);
				mode = 2;
			}
			initCount++;
		}
		else if((mode != 2 || mode != 5) && cos(distToCP[0].p) < 0 && cos(distToCP[1].p) > 0) {
			cos(relHeading) > 0 ? mode = 2 : mode = 5;
		}
		else if((mode != 3 || mode != 4) && cos(distToCP[0].p) > 0) {mode = 3;}
		else if((mode != 6 || mode != 7) && cos(distToCP[1].p) < 0) {mode = 6;}
		else if(mode == 3){
			sin(relHeading) > 0 && cos(relHeading) < 0 ? mode += 1 : 0;
		}
		else if(mode == 6){
			sin(relHeading) > 0 && cos(relHeading) > 0 ? mode += 1 : 0;
		}
	}
	else{
		mode = -1;
	}
	Serial.print(",mode:,"); Serial.println(mode);
	//Serial.print(",relHead:,"); Serial.print(relHeading);
	//Serial.print(",HeadtoCp0:,"); Serial.print(distToCP[0].p);
	//Serial.print(",HeadtoCp1:,"); Serial.println(distToCP[1].p);
	return mode;
}

/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨーレートフィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControlPID(int8_t mode, float value, float yawRate,float targetYawRt)
{
	static unsigned long lastTime;
	float sampleTime = T10.getInterval() * 0.001;
#ifdef CC01
	float kp = 0.6 * 3,ti = 0.5 * 0.42 ,td = 0.125 * 0.42,diff;
#else
	float kp = 2.125,ti = 0.4 ,td = 0.1,diff;
#endif
	static float err,lastyawRate;
	static int8_t lastMode;

	lastTime ? sampleTime = (millis() - lastTime) * 0.001 : 0;

	if(mode != lastMode){
		err = 0;
		lastyawRate = 0;
	}
	!value ? value = 90 : 0;
	err  += (targetYawRt - yawRate) * sampleTime;
	diff = 	(yawRate - lastyawRate) / sampleTime;

  value +=  kp * (err/ti - (yawRate + td * diff));

	value = LimitValue(value,120,60);

	lastyawRate = yawRate;
	lastMode = mode;
	lastTime = millis();
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
 float StrControlPIDAng(int8_t mode, float value, float yawAngle,float targetYawAngle)
 {
	static unsigned long lastTime;
	float sampleTime = T10.getInterval() * 0.001;
 #ifdef CC01
  float kp = 180,ti = 0.5 * 0.42 ,td = 0.125 * 0.42,diff;
 #else
 	float kp = 2.125,ti = 0.4 ,td = 0.1,diff;
 #endif
 	static float err[3],lastyawAngle,lastvalue;
 	static int8_t lastMode;

	lastTime ? sampleTime = (millis() - lastTime) * 0.001 : 0;
 	if(mode != lastMode){
 		err[0]=0;err[1]=0;err[2]=0;
 		lastyawAngle = 0;
 	}
 	!value ? value = 90 : 0;
	err[2] = err[1];
	err[1] = err[0];
 	err[0] = (targetYawAngle - yawAngle);
 	diff = kp * (err[0] - err[1] + err[0] * sampleTime / ti + td / sampleTime * (err[0]-2*err[1]+err[2]));

  lastvalue ? value =  lastvalue + diff : 0;

 	value = LimitValue(value,120,60);
	lastvalue = value;
 	lastyawAngle = yawAngle;
 	lastMode = mode;
	lastTime = millis();
 #if 0
 	//#ifdef DEBUG
 	Serial.print("Time,");Serial.print(millis());
 	Serial.print(",err:,"); Serial.print(err[0]); Serial.print(",YawAng:,"); Serial.print(yawAngle);
	Serial.print(",Diff:,"); Serial.print(diff);
 	Serial.print(",value:,"); Serial.println(value);
 	//#endif
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
    return fmod(RoundRad(x) + M_PI, 2 * M_PI) - M_PI;
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
