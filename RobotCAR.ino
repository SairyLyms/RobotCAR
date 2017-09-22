/************************************************************************/
/*	Include Files														*/
/************************************************************************/
#include <NMEAGPS.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include <Servo.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <complex.h>
#include "Param.h"
/************************************************************************/
/*	Macro Definition													*/
/************************************************************************/
#define   R 0.61803399    /* 黄金分割比 golden section ratio */
#define   C (1.0 - R)
#define   SHFT2(a,b,c)  (a)=(b); (b)=(c);
#define   SHFT3(a,b,c,d)  (a)=(b); (b)=(c); (c)=(d);
/************************************************************************/
/*	Struct Definition													*/
/************************************************************************/
/************************************************************************/
/*	Private Constant Definition											*/
/************************************************************************/
int8_t n = 5;

// ================================================================
// ===                       Course Data                        ===
// ================================================================
polVectorFloat3D distToCP[2];


//#define DEBUG_IMU
//#define DEBUG_GPS
//#define DEBUG
//#define TechCom											/*テストコース設定*/
//#define Ground
//#define Garden
#define HappiTow
//#define Home
#ifdef TechCom
NeoGPS::Location_t cp1(365759506L,1400158539L);
NeoGPS::Location_t cp2(365760193L,1400160370L);
#elif defined Garden
NeoGPS::Location_t cp1(365833090L,1400104240L);
NeoGPS::Location_t cp2(365832140L,1400104870L);
#elif defined HappiTow
NeoGPS::Location_t cp2(365680690L,1399957580L);
NeoGPS::Location_t cp1(365679700L,1399957730L);
//NeoGPS::Location_t cp0(365680389L,1399960780L);
//NeoGPS::Location_t cp1(365679436L,1399957780L);
#elif defined Home
NeoGPS::Location_t cp1(385071520L,1403969840L);
NeoGPS::Location_t cp2(385071540L,1403970570L);
#elif defined Ground
NeoGPS::Location_t cp1(385156170L,1403968200L);
NeoGPS::Location_t cp2(385155440L,1403966060L);
#endif
NeoGPS::Location_t cp[3][3];
//NeoGPS::Location_t locCenter((long)(5*(0.1 * cp[0].lat() + 0.1 * cp[1].lat())),(long)(5*(0.1 * cp[0].lon()+ 0.1 * cp[1].lon())));

NeoGPS::Location_t relPos2D;
volatile NMEAGPS  gps;
volatile gps_fix  fix;
MPU6050 IMU;
HMC5883L mag;
Madgwick AHRS;
Servo FStr,PowUnit;
Scheduler runner;
/*座標系は安部先生の本に従う(右手系)*/
VectorFloat rpyAngle;
VectorFloat rpyRate;
VectorFloat acc;
volatile float rfromCenter;
volatile float heading;											/*方位(rad,deg)*/
volatile float gpsSpeedmps;										/*GPS絶対速度(m/s)*/
double pos[2][3] = {{0,0,0},{0,0,0}};
float directioncp;
float relHeading;							 					/*CP間中心線からの相対角度(Heading:CCW+)*/
float relYawAngle;												/*CPまでの角度(Heading:CCW+)*/
float headingOffstIMU;
polVectorFloat3D centerPos;
uint8_t sats;
uint8_t overCP;
float puPwm = 90;														/*パワーユニットのPWM*/
float fStrPwm = 90;													/*ステアリングのPWM*/
uint8_t sampleTimems = 10;
int8_t mode,imuCalibMode;
// ================================================================
// ===               				Prototype Def.			                ===
// ================================================================
void Task1(void);
void Task2(void);
void Task5(void);
void Task10(void);
// ================================================================
// ===               						Tasks						                ===
// ================================================================
Task T1(sampleTimems, TASK_FOREVER, &Task1, &runner,true);
Task T2(sampleTimems * 2, TASK_FOREVER, &Task2, &runner,true);
Task T10(sampleTimems * 10, TASK_FOREVER, &Task10, &runner,true);
// ================================================================
// ===                     Course setup		                    ===
// ================================================================
void SetWaypoint(float r)
{
  directioncp = cp2.BearingTo(cp1);
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      if(i==0){cp[i][j].lat(0.5 * cp1.lat() + 0.5 * cp2.lat());cp[i][j].lon(0.5 * cp1.lon() + 0.5 * cp2.lon());}
      else if(i==1){cp[i][j].lat(cp1.lat());cp[i][j].lon(cp1.lon());}
      else if(i==2){cp[i][j].lat(cp2.lat());cp[i][j].lon(cp2.lon());}
      }
    }
  cp[1][1].OffsetBy( r * 0.001 / NeoGPS::Location_t::EARTH_RADIUS_KM , M_PI*(0.5) + directioncp); //cp1方向右側
  cp[1][2].OffsetBy( r * 0.001 / NeoGPS::Location_t::EARTH_RADIUS_KM , M_PI*(1.5) + directioncp); //cp1方向左側
  cp[2][1].OffsetBy( r * 0.001 / NeoGPS::Location_t::EARTH_RADIUS_KM , M_PI*(0.5) + directioncp);
  cp[2][2].OffsetBy( r * 0.001 / NeoGPS::Location_t::EARTH_RADIUS_KM , M_PI*(1.5) + directioncp);
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
	Wire.begin();
	Serial.begin(115200);
	Serial1.begin(38400);

	IMU.setI2CMasterModeEnabled(false);
	IMU.setI2CBypassEnabled(true) ;
	IMU.setSleepEnabled(false);
	IMU.initialize();
	mag.initialize();

	IMU.setXAccelOffset(451);IMU.setYAccelOffset(-3769);IMU.setZAccelOffset(634);
	IMU.setXGyroOffset(105);IMU.setYGyroOffset(-21);IMU.setZGyroOffset(16);

	FStr.attach(9,1000,2000);
	PowUnit.attach(8,1000,2000);

	Serial.flush();
	SetWaypoint(2.0); //旋回半径設定
	runner.startNow();
	Initwait();
}

/************************************************************************
 * FUNCTION : 1周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task1(void)
{
	IMUupdate();
	IntegratedChassisControl();
}

/************************************************************************
 * FUNCTION : 2周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task2(void)
{
	ChassisFinalOutput(puPwm,fStrPwm);
}

/************************************************************************
 * FUNCTION : 10周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Task10(void)
{
	GPSupdate();
}

/************************************************************************
 * FUNCTION : GPS更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void GPSupdate(void)
{
	if(gpsSpeedmps){gpsSpeedmps = fix.speed() * 0.514444;}
	if(heading){relHeading = Pi2pi(heading + directioncp);}
	if(rfromCenter) {
		pos[0][0] = rfromCenter * cos(-(fix.location.BearingTo(cp[0][0]) - directioncp)) * -1;
		pos[0][1] = rfromCenter * sin(-(fix.location.BearingTo(cp[0][0]) - directioncp)) * -1;
	}
}


/************************************************************************
 * FUNCTION : GPS_Fix後更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
 void GPSFIX(void)
 {
	 if(fix.valid.speed){gpsSpeedmps = fix.speed() * 0.514444;}
	 if(fix.valid.heading){heading = -fix.heading() * 0.01745329251;}
	 if(fix.valid.location) {rfromCenter = fix.location.DistanceKm(cp[0][0]) * 1000.0f;}
 }

float ave(float in,int n)//n回平均計算(n回計算後に初めて出力)
{
	static int i;
	static float buf;
	float ave = 0, w = 1.0/n;
	if(i < n){
		buf += in * w;
		i++;
	}
	else{
		ave = buf;
		buf = 0;
		i = 0;
	};
	return ave;
}

float aveAngle(float in,int n)//n回平均計算(n回計算後に初めて出力)
{
	static int i;
	static float bufx,bufy;
	float ave = 0, w = 1.0/n;
	if(i < n){
		bufx += cos(in);
		bufy += sin(in);
		i++;
	}
	else{
		ave = atan2(bufy,bufx);
		bufx = 0;
		bufy = 0;
		i = 0;
	};
	return ave;
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
	float mfx, mfy, mfz;
	int aix, aiy, aiz;
	int gix, giy, giz;
	int mix, miy, miz;

	IMU.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);
	mag.getHeading(&mix, &miy, &miz);

	afx = convertRawAcceleration(aiy);
	afy = -convertRawAcceleration(aix);
	afz = convertRawAcceleration(aiz);

	gfx = convertRawGyro(giy);
	gfy = -convertRawGyro(gix);
	gfz = convertRawGyro(giz);

	mfx = (float)mix;
	mfy = (float)miy;
	mfz = (float)miz;

	AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);

	rpyRate.x = gfx * M_PI/180;
	rpyRate.y = gfy * M_PI/180;
	rpyRate.z = gfz * M_PI/180;

	rpyAngle.x = AHRS.getRollRadians(); //ロール角
	rpyAngle.y = AHRS.getPitchRadians(); //ピッチ角
	rpyAngle.z = AHRS.getYawRadians(); //ヨー角
	relYawAngle = Pi2pi(rpyAngle.z + headingOffstIMU);

	acc.x = afx;
	acc.y = afy;
	acc.z = afz;

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
 * FUNCTION : 開始入力待ち処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void Initwait(void)
{
	Serial.println(F("Send any character to Start learning direction!!: "));	
	while (Serial.available() && Serial.read()) ; // empty buffer
	while (!Serial.available()) ;          // wait for data
	while (Serial.available() && Serial.read()) ; // empty buffer
}

/************************************************************************
 * FUNCTION : サーボ出力(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void ChassisFinalOutput(float puPwm,float fStrPwm)
{
	PowUnit.write((int)puPwm);
	FStr.write(180 - (int)fStrPwm);
}


/************************************************************************
 * FUNCTION : シャシ統合制御(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void IntegratedChassisControl(void)
{
	StateManager();
	switch (mode) {
	case 0:		fStrPwm = StrControlPIDnew(rpyRate.z,0.0);	
				puPwm = 90;
				//puPwm = LimitValue(SpdControlPID(gpsSpeedmps,1.0),120,90);
				break;
	//1:ヨー角キャリブレーション用
	case 1:		fStrPwm = StrControlPIDnew(rpyRate.z,0.0);	
				puPwm = 107;
				break;
	case 2:		fStrPwm = 90;
				puPwm = 90;
				break;
	case 3:		puPwm = 107;//SpdControlPID(gpsSpeedmps,2.0);
				ClothoidControl();
				break;
	case 0xf:	puPwm = 90;
				fStrPwm = StrControlPIDnew(rpyRate.z,0.0);
				break;

	default:	puPwm = 90;
				break;
	}
	#if 0
	Serial.print(",mode,");Serial.print(mode);
	Serial.print(",IMUYaw,");Serial.print(RoundRadPosNeg(rpyAngle.z));
	Serial.print(",headingIMU,");Serial.print(RoundRadPosNeg(rpyAngle.z + headingOffstIMU - headingOffst));
	Serial.print(",Heading,");Serial.print(heading);
	Serial.print(",CP0,");Serial.print(RoundRadPosNeg(distToCP[0].t - headingOffst));
	Serial.print(",CP1,");Serial.print(RoundRadPosNeg(distToCP[1].t - headingOffst + M_PI));
	Serial.print(",overCP,");Serial.print(overCP);
	Serial.println("");
	//Serial.print(",RoundRadPosNeg");Serial.println(RoundRadPosNeg(rpyAngle.z-headingOffstIMU+headingOffst));
	#ifdef DEBUG
	//Serial.print(",TgtAngle:,"); Serial.print(targetAngleCP);
	//Serial.print(",TgtYawRt:,"); Serial.print(targetYawRtCP);Serial.print(",yawRate:,"); Serial.print(rpyRate.z,6); Serial.print(",Speed:,"); Serial.print(gpsSpeedmps);
	//Serial.print(",Mode:,"); Serial.print(mode);Serial.print("StrPWM:,"); Serial.println(fStrPwm);
	#endif
	#endif
}

/************************************************************************
 * FUNCTION : クロソイド曲線走行制御(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void ClothoidControl(void)
{	
	static int controlMode = 0;
	static double cvRate,cvOffset,odoEnd,odo;
	double yawRt;
	Serial.print("Current Mode :");Serial.println(controlMode);
	if(controlMode==0){
		float l = fix.location.DistanceKm(cp[1][1]) * 1000.0f, psi = -Pi2pi(fix.location.BearingTo(cp[1][1]) - directioncp);//cp1右側へ
		CalcClothoidCurvatureRate(l,psi,relYawAngle,0,&cvRate,&cvOffset,&odoEnd,5);
		controlMode = 1;
	}
	else if(controlMode==1){//初回cp1旋回開始
		odo += gpsSpeedmps * sampleTimems * 0.001;
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPIDnew(rpyRate.z,yawRt);
		if(odo>odoEnd){
			Serial.println("Control1 done!!:,");//制御終了して次の制御へ
			float l = fix.location.DistanceKm(cp[1][2]) * 1000.0f, psi = -Pi2pi(fix.location.BearingTo(cp[1][2]) - directioncp);//cp1反対側へ定常円
			CalcClothoidCurvatureRate(l,psi,relYawAngle,M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp1反対側へ定常円			
			odo = 0;
			controlMode = 2;
		}
	}
	else if(controlMode==2){//cp1定常円
		odo += gpsSpeedmps * sampleTimems * 0.001;
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPIDnew(rpyRate.z,yawRt);
		if(odo>odoEnd){
			Serial.println("Control2 done!!:,");//制御終了して次の制御へ
			float l = fix.location.DistanceKm(cp[2][1]) * 1000.0f, psi = -Pi2pi(fix.location.BearingTo(cp[2][1]) - directioncp - M_PI);//cp2へ
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),0,&cvRate,&cvOffset,&odoEnd,5);//cp2へ						
			odo = 0;
			controlMode = 3;
		}
	}
	else if(controlMode==3){//cp2へ
		odo += gpsSpeedmps * sampleTimems * 0.001;
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPIDnew(rpyRate.z,yawRt);
		if(odo>odoEnd){
			Serial.println("Control3 done!!:,");//制御終了して次の制御へ
			float l = fix.location.DistanceKm(cp[2][2]) * 1000.0f, psi = -Pi2pi(fix.location.BearingTo(cp[2][2]) - directioncp - M_PI);//cp2反対側へ定常円
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),-3.14,&cvRate,&cvOffset,&odoEnd,5);//cp2反対側へ定常円									
			odo = 0;
			controlMode = 4;
		}
	}
	else if(controlMode==4){//cp2定常円
		odo += gpsSpeedmps * sampleTimems * 0.001;
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPIDnew(rpyRate.z,yawRt);
		if(odo>odoEnd){
			Serial.println("Control4 done!!:,");//制御終了して次の制御へ
			float l = fix.location.DistanceKm(cp[1][1]) * 1000.0f, psi = -Pi2pi(fix.location.BearingTo(cp[1][1]) - directioncp);//cp1右側へ
			CalcClothoidCurvatureRate(l,psi,relYawAngle,0,&cvRate,&cvOffset,&odoEnd,5);//cp1右側へ
			odo = 0;
			controlMode = 5;
		}
	}
	else if(controlMode==5){//cp1へ
		odo += gpsSpeedmps * sampleTimems * 0.001;
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPIDnew(rpyRate.z,yawRt);
		if(odo>odoEnd){
			Serial.println("Control5 done!!:,");//制御終了して次の制御へ
			float l = fix.location.DistanceKm(cp[1][2]) * 1000.0f, psi = -Pi2pi(fix.location.BearingTo(cp[1][2]) - directioncp);//cp1反対側へ定常円
			CalcClothoidCurvatureRate(l,psi,relYawAngle,M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp1反対側へ定常円	
			odo = 0;
			controlMode = 2;
		}
	}
	Serial.print("odo:,");Serial.print(odo);Serial.print("TgtYaw:,");Serial.println(yawRt);
}

/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    :
 *
 * OUTPUT   : 状態値
 ***********************************************************************/
int8_t StateManager()
{
	static unsigned long startTime = 0;
	if(rfromCenter < 20) {			//コース中央から半径20m内
		if(mode <= 0){
			Serial.println("1:Start,d:debug");
			while(Serial.available()){
				switch(Serial.read()){
					case '1'	: mode = 1;break; //再度IMU補正
					case 'd'	: mode = 0xf;break; //走行開始
					default		: break;	
				}
			}
		}
		else if(mode == 1 && heading){
			if(!imuCalibMode){startTime = millis();imuCalibMode = 1;}	//助走モード開始(1)
			else if(imuCalibMode == 1 && (millis() - startTime) > 10000){
				Serial.println("Calib Start");
				imuCalibMode = 2;										//学習開始(2)
			}
			else if(imuCalibMode == 2){
				headingOffstIMU = -aveAngle(rpyAngle.z,50);
				if(headingOffstIMU){
					headingOffstIMU += relHeading; 
					imuCalibMode = 3;
					mode = 2;
					Serial.println("Calib Done");						//学習完了(3)
				}
			}
		}
		else if(mode == 2){
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.println(F(" ,To start, put key '1' , To learning direction, put Key '0'"));
			//Serial.print("Heading,");Serial.print(heading);Serial.print("RelHead,");Serial.print(relHeading);Serial.print("RelYaw:,"); Serial.println(relYawAngle);			
			while(Serial.available()){
				switch(Serial.read()){
					case '0'	: mode = 1;imuCalibMode = 0;break; //再度IMU補正
					case '1'	: mode = 3;break; //走行開始
					default		: break;	
				}
			}
		}
		else if(mode == 3){
			//Serial.println("Now Mode 3");
		}
	}
	else{//コース外
		mode = -1;
		//Serial.println("Out of Course");
	}
	//Serial.print(",mode:,"); Serial.println(mode);
	//Serial.print(",relHead:,"); Serial.print(relHeading);
	//Serial.print(",HeadtoCp0:,"); Serial.print(distToCP[0].p);
	//Serial.print(",Headtocp0:,"); Serial.println(distToCP[1].p);
}

/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨーレートフィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControlPID(int mode, float value, float yawRate,float targetYawRt)
{
	static unsigned long lasttimems;
	float sampleTime = 10 * 0.001;
	float kp = 2.125,ti = 0.4 ,td = 0.1,diff;
	static float err,lastyawRate;
	static int lastMode;

	if(mode != lastMode){
		err = 0;
		lastyawRate=0;
	}

	!value ? value = 90 : 0;
	err  += (targetYawRt - yawRate) * sampleTime;
	diff = 	(yawRate - lastyawRate) / sampleTime;

  	value +=  kp * (err/ti - (yawRate + td * diff));

	value = LimitValue(value,120,60);

	lastyawRate = yawRate;
	lastMode = mode;

	//Serial.print("Time,");Serial.print(millis());
	//Serial.print(",err:,"); Serial.print(err);Serial.print(",diff:,"); Serial.print(diff); Serial.print(",TGTYawRt:,"); Serial.print(targetYawRt); Serial.print(",YawRt:,"); Serial.print(rpyRate.z);
	//Serial.print(",value:,"); Serial.println(value);

	return value;

}

/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨー角フィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
 float StrControlPIDnew(float currentYawAngle,float targetYawAngle)
 {
	float KP = 30,TI = 0.105,TD = 0.02625,bias = 90;
	static float error_prior = 0,integral = 0;
	float error,derivative,output,sampleTime = 10 * 0.001;

	error = targetYawAngle - currentYawAngle;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = KP * (error + integral / TI + TD * derivative) + bias;

	error_prior = error;
	
	return output;
 }

/************************************************************************
 * FUNCTION : 速度コントロール
 * INPUT    :
 *
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float SpdControlPID(float currentSpeed,float targetSpeed)
{
	float KP = 8,TI = 0.5,TD = 0.125,bias = 90;
	static float error_prior = 0,integral = 0;
	float error,derivative,output,sampleTime = 10 * 0.001;

	error = targetSpeed - currentSpeed;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = KP * (error + integral / TI + TD * derivative) + bias;
	Serial.print("outputraw:");Serial.println(output);
	output = LimitValue(output,140,80);
	error_prior = error;
	Serial.print("Time:");Serial.print(millis());Serial.print("Speed:");Serial.print(gpsSpeedmps);
	Serial.print("error:");Serial.print(error);Serial.print("output:");Serial.println(output);

	return output;
}

/************************************************************************
 * FUNCTION : 1Dテーブルルックアップ関数
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
 float Lookup1D(float value,float tableX[],float tableK[],int8_t tableSize)
 {
	 float buf;
	 int sign = (value > 0) - (value < 0);
	 if(tableSize > 2){
	 for(int i=0;i<tableSize;i++){
		 if(abs(value) <= tableX[0]){
				buf =  tableK[0] * tableX[0];
			 }
		 else if(abs(value) >= tableX[tableSize-1]){
				buf = sign * tableK[tableSize-1] * tableX[tableSize-1];
			}
		 else{
			 if(abs(value) > tableX[i-1] && abs(value) < tableX[i+1])
			 {
				buf = value * tableK[i];
			 }
		 }
	 }
	 return buf;
 }
}

/************************************************************************
 * FUNCTION : コース設定
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
 void GetLenAndPsi(double x0, double x1, double y0, double y1,double *len, double *psi)
 {
	 *len = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
	 *psi = atan2f((y1-y0),(x1-x0));
 }



/************************************************************************
 * FUNCTION : クロソイド曲線用関数群
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
double Phi(double phi0, double phiV, double phiU, double S)
{
return phi0 + phiV * S + phiU * S * S;
}

Complex slope(double phi0,double phiV, double phiU, double S)
{
  Complex I(0,Phi(phi0, phiV, phiU, S));
  return I.c_exp();
}

int CalcParamClothoid(double phi0, double phi1 ,double phiV,double *psi,double *lambda ,int8_t n)
{
  Complex integral = (0,0),dA = (0,0); // 積分結果
  double w = 1/(double)n; // 積分範囲を n 個に分割したときの幅
  double S,phiUCalc;
  // === Simpson 法による積分 (開始） ===
  w = 1 / (double)n; // 分割幅
  S = 0;
  phiUCalc = phi1 - phi0 - phiV;
  for (int8_t i=0; i<n; i++) {
      dA = (slope(0, phiV, phiUCalc, S) + slope(0, phiV, phiUCalc, S+w)) * 0.5 * w;
      integral += dA;// Σ
      S += w;
  }
*psi = Pi2pi(integral.phase());
*lambda = integral.modulus();
return 0;
}

float CompMinPsi(double phi0, double phi1 ,double phiV , double targetpsi,double *lambda , int8_t n)
{
    double psi,x;
    CalcParamClothoid(phi0,phi1,phiV,&psi,lambda,n);
    Complex I1(0,psi),I2(0,(targetpsi - phi0));
    x = (I1.c_exp() - I2.c_exp()).modulus();
	  return x;
}

float CalcphiV(double (*f)(double,double,double,double,double *,int8_t), double *xmin, double phi0, double phi1 , double targetpsi, double *lambda, int8_t n)
{
	double f1, f2, x0, x1, x2, x3;
    double ax = -6.28;
    double bx = 6.28;
	x0 = ax;							/* ４点 x0,x1,x2,x3 を更新していく */
	x1 = ax + C * (bx - ax);
	x2 = ax + R * (bx - ax);
	x3 = bx;

	f1 = (*f)(phi0,phi1,x1,targetpsi,lambda,n);						/* 関数の最初の評価 */
	f2 = (*f)(phi0,phi1,x2,targetpsi,lambda,n);
	while(fabs(x3 - x0) > 0.01 * (fabs(x1) + fabs(x2)))	/* 収束判定 */
	{
		if(f2 < f1)								/* 場合分けの一方 */
		{
			SHFT3(x0, x1, x2, R * x1 + C * x3)	/* 各点の更新 */
			SHFT2(f1, f2, (*f)(phi0,phi1,x2,targetpsi,lambda,n))				/* 関数を評価 */
		}
		else									/* 場合分けのもう一方 */
		{
			SHFT3(x3, x2, x1, R * x2 + C * x0)
			SHFT2(f2, f1, (*f)(phi0,phi1,x1,targetpsi,lambda,n))				/* 関数を評価 */
		}
	}
	if(f1 < f2)				/* 完成。最新の２点のうち良い方を返す */
	{
		*xmin = x1;
		return f1;
	}
	else
	{
		*xmin = x2;
		return f2;
	}
}

void CalcClothoidCurvatureRate(double l, double psi, double phi0,double phi1, double *cvRate,double *cvOffset,double *odoEnd,int n)
{
double w; // 積分範囲を n 個に分割したときの幅
double S,lambda,phiV,phiU,h,odo[n],cv[n];
int i;
// === Simpson 法による積分 (開始） ===
w = 1.0 / (float)(n); // 分割幅
S = 0;
CalcphiV(CompMinPsi,&phiV,phi0,phi1,psi,&lambda,n);
phiU = phi1 - phi0 - phiV;
h = l / lambda;
for (i=0; i<n; i++) {
cv[i] = (phiV + 2 * phiU * S)/h;
S += w;
odo[i] = h*S;
}
*cvRate = (cv[n-1] - cv[0])/(odo[n-1]-odo[0]);
*cvOffset = cv[0];
*odoEnd = odo[n-1];
}

double calcYawRt(double v,double odo,double cvRate,double cvOffset,double odoend)
{
    /*入力    :  速度v,現在距離odo,曲率距離比cvRate,距離ゼロ時曲率*/
    /*出力    :  ヨーレート*/
    double yawRt;
    if(odo >= 0 && odo < odoend){yawRt = v * ((odo * cvRate) + cvOffset);}
    else if(odo >= odoend){yawRt = v * ((odoend * cvRate) + cvOffset);}
    else{yawRt = v * cvOffset;}
    return yawRt;
}

double odometry(double v,int mode)
{
    static double odo = 0;
    static int lastMode = 0;
    static unsigned long lastTime;  /*millis*/
    if(lastMode != mode){odo = 0;}
    else{
        odo += v * (millis()-lastTime) * 0.001;
    }
    lastMode = mode;
    lastTime = millis();
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

float Pi2pi(float angle)
{
    while(angle >= M_PI) {angle -= M_PI * 2;}
    while(angle < -M_PI){angle += M_PI * 2;}
    return angle;
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
		fix = gps.read();
	}
	GPSFIX();
}
