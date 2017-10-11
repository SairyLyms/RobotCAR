/************************************************************************/
/*	Include Files														*/
/************************************************************************/
#include <NMEAGPS.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
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
float wheelbase = 0.266;
float kStrAngle2Pwm = 191.2;
float strPwmOffset = 88.38;
int8_t n = 5;
float lengthCenterToWaypoint = 7.0f;
float r = 3.0f;
// ================================================================
// ===                       Course Data                        ===
// ================================================================
//#define DEBUG_IMU
//#define DEBUG_GPS
//#define DEBUG
//#define TechCom											/*テストコース設定*/
//#define Ground
//#define Garden
#define HappiTow
//#define Home
#ifdef TechCom
NeoGPS::Location_t cp0(365759506L,1400158539L);
NeoGPS::Location_t cp1(365760193L,1400160370L);
#elif defined Garden
NeoGPS::Location_t cp1(365833090L,1400104240L);
NeoGPS::Location_t cp2(365832140L,1400104870L);
#elif defined HappiTow
NeoGPS::Location_t cp1(365680770L,1399958340L);
NeoGPS::Location_t cp2(365679700L,1399958340L);
#elif defined Home
NeoGPS::Location_t cp1(385071520L,1403969840L);
NeoGPS::Location_t cp2(385071540L,1403970570L);
#elif defined Ground
NeoGPS::Location_t cp1(385156170L,1403968200L);
NeoGPS::Location_t cp2(385155440L,1403966060L);
#endif
NeoGPS::Location_t locCenter;
Madgwick AHRS;
volatile NMEAGPS  gps;
volatile gps_fix  fix;
MPU6050 IMU;
Servo FStr,PowUnit;
Scheduler runner;
/*座標系は安部先生の本に従う(右手系)*/
VectorFloat rpyAngle;
float relYawAngle = 0;									/*CPまでの角度(Heading:CCW+)*/
VectorFloat rpyRate;
VectorFloat acc;
volatile float rfromCenter,bearfromCenter;
volatile float heading;											/*方位(rad,deg)*/
volatile float gpsSpeedmps;										/*GPS絶対速度(m/s)*/
float directioncp;
volatile float relHeading,GPSyawRt;							 	/*CP間中心線からの相対角度(Heading:CCW+)*/
float headingOffstIMU;
float puPwm = 90;												/*パワーユニットのPWM*/
float fStrPwm = strPwmOffset;									/*ステアリングのPWM*/
float SampleTime10ms = 10.0f;
uint8_t sampleTimems = 10;
int8_t mode,imuCalibMode;
// ================================================================
// ===              	Prototype Def.			                ===
// ================================================================
void Task1(void);
void Task2(void);
void Task5(void);
void Task10(void);
// ================================================================
// ===               		Tasks				                ===
// ================================================================
Task T1(sampleTimems, TASK_FOREVER, &Task1, &runner,true);
Task T2(sampleTimems * 2, TASK_FOREVER, &Task2, &runner,true);
// ================================================================
// ===                     Course setup		                    ===
// ================================================================
void SetWaypoint(void)
{
  directioncp = cp2.BearingTo(cp1);
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
	Wire.begin();
	Wire.setClock(400000L);
	Serial.begin(115200);
	Serial1.begin(38400);

	IMU.initialize();	
	IMU.setSleepEnabled(false);
	IMU.setDLPFMode(MPU6050_DLPF_BW_5); 
	IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_500); 
	IMU.setRate(9);
	IMU.setXAccelOffset(478);IMU.setYAccelOffset(-3781);IMU.setZAccelOffset(585);
	IMU.setXGyroOffset(74);IMU.setYGyroOffset(-43);IMU.setZGyroOffset(32);

	FStr.attach(9,1000,2000);
	PowUnit.attach(8,1000,2000);

	locCenter.lat(5*(0.1 * cp1.lat() + 0.1 * cp2.lat()));
	locCenter.lon(5*(0.1 * cp1.lon() + 0.1 * cp2.lon()));

	Serial.flush();
	SetWaypoint(); //旋回半径設定
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
	HeadingUpdateIMU(&heading, &relHeading, rpyRate.z, SampleTime10ms);
	IntegratedChassisControl();
	GetSampleTime10ms(&SampleTime10ms);
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
 * FUNCTION : GPS_Fix後更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
 void GPSFIX(void)
 {
	 if(fix.valid.speed){gpsSpeedmps = fix.speed() * 0.514444;}
	 if(fix.valid.heading){
		 heading = -fix.heading() * 0.01745329251;
		 relHeading = Pi2pi(heading + directioncp);
		}
	 if(fix.valid.location) {
		NeoGPS::Location_t current(fix.location.lat(),fix.location.lon());
		rfromCenter = fix.location.DistanceKm(locCenter) * 1000.0f;
		bearfromCenter = -Pi2pi(locCenter.BearingTo(current) - directioncp);
	}
 }

/************************************************************************
 * FUNCTION : 角度のn回平均計算
 * INPUT    : 角度(rad),回数
 * OUTPUT   : n回平均角度(rad)
 ***********************************************************************/
float aveAngle(float in,int n)
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

float LimitValue(float inputValue,float lowerLimitValue,float upperLimitValue)
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
	IMU.resetFIFO();
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
	case 0:		fStrPwm = strPwmOffset;//StrControlPID(rpyRate.z,0.0);	
				puPwm = 90;
				break;
	//1:ヨー角キャリブレーション用
	case 1:		fStrPwm = StrControlPID(rpyRate.z,0.0);	
				puPwm = 107;
				break;
	case 2:		fStrPwm = 90;
				puPwm = 90;
				break;
	case 3:		puPwm = SpdControlPID(gpsSpeedmps,2.5f);
				ClothoidControl();
				break;
	case 4:     if(gpsSpeedmps > 0.5f){puPwm = SpdControlPID(gpsSpeedmps,0.0f);}
				else{puPwm = 90;}
				break;
	case 0xf:	
				if(Serial.available()){
					switch(Serial.read()){
						case '0'	: fStrPwm = 90;break;
						case '1'	: fStrPwm -= 2;break; //再度IMU補正
						case '2'	: fStrPwm += 2;break; //走行開始
						//case '3'	: fStrPwm = 70;break; //CW
						//case '4'	: fStrPwm = 104;break; //CCW
						case 's'	: puPwm = 110;break; //走行開始
						case 'r'	: headingOffstIMU = -rpyAngle.z;
						default		: puPwm = 90;break;	
					}
				}
				Serial.print("Strpwm,");Serial.print(fStrPwm);Serial.print(",");
				Serial.print("yawAng,");Serial.print(relYawAngle * 180/M_PI);Serial.print(",");
				Serial.print("Heading, ");Serial.print(heading);Serial.print(",");
				Serial.print("x,");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print(",");
				Serial.print("y,");Serial.print(rfromCenter * sin(bearfromCenter));Serial.print(",");
				Serial.print("yawRate,");Serial.print(rpyRate.z);Serial.print(",");
				Serial.print("Speed,");Serial.print(gpsSpeedmps);Serial.print(",");
				Serial.println("");
				break;
	default:	puPwm = 90;
				break;
	}
}

/************************************************************************
 * FUNCTION : クロソイド曲線走行制御(テスト中)
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void ClothoidControl(void)
{	
	static int controlMode = 0;
	static float cvRate,cvOffset,odoEnd,odo;
	float yawRt;
	float l,psi;
	uint8_t f_Timer = 0;
	//Serial.print("Current Mode :");Serial.print(controlMode);
	//Serial.print("psi:,");Serial.print(psi);
	f_Timer = TimerSec(165); //カウントダウンタイマ作動
	if(controlMode==0){
		GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,-r,&l,&psi);
		CalcClothoidCurvatureRate(l,psi,relYawAngle,0,&cvRate,&cvOffset,&odoEnd,5);
		controlMode = 1;
	}
	else if(controlMode==1){//初回cp1旋回開始
		//yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("relHeading:");Serial.println(relHeading);
		fStrPwm  = calcStrpwm(odo,cvRate,cvOffset,odoEnd);//StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;
		if(odo>odoEnd){
			Serial.println("Control1 done!! Next2:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,relHeading,M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp1反対側へ定常円			
			//CalcClothoidCurvatureRate(l,psi,relYawAngle,M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp1反対側へ定常円
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));			
			Serial.print("len:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("cvRate:");Serial.print(cvRate);
			Serial.print("cvOffset:");Serial.print(cvOffset);Serial.print("odoEnd:");Serial.println(odoEnd);				
			odo = 0;
			controlMode = 2;
		}
	}
	else if(controlMode==2){//cp1定常円
		//yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = calcStrpwm(odo,cvRate,cvOffset,odoEnd);//StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control2 done!! Next3:,");//制御終了して次の制御へ
			GetLen(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),0,0,&l);
			GetPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),-lengthCenterToWaypoint,-r,&psi);
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relHeading-M_PI),psi,&cvRate,&cvOffset,&odoEnd,5);//cp0へ
			//CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),psi,&cvRate,&cvOffset,&odoEnd,5);//cp0へ
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));
			Serial.print("len:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("cvRate:");Serial.print(cvRate);
			Serial.print("cvOffset:");Serial.print(cvOffset);Serial.print("odoEnd:");Serial.println(odoEnd);							
			odo = 0;
			controlMode = 3;
		}
	}
	else if(controlMode==3){//cp0へ
		Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("relHeading:");Serial.println(relHeading);
		fStrPwm  = calcStrpwm(odo,cvRate,cvOffset,odoEnd);//StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control3 done!! Next4:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),-lengthCenterToWaypoint,-r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relHeading-M_PI),0,&cvRate,&cvOffset,&odoEnd,5);//cp2へ
			//CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),0,&cvRate,&cvOffset,&odoEnd,5);//cp2へ
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));
			Serial.print("len:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("cvRate:");Serial.print(cvRate);
			Serial.print("cvOffset:");Serial.print(cvOffset);Serial.print("odoEnd:");Serial.println(odoEnd);							
			odo = 0;
			controlMode = 4;
			}
		else if(f_Timer && odo > odoEnd-2){
			mode = 4;
		}
	}
	else if(controlMode==4){//cp2へ
		//yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("relHeading:");Serial.println(relHeading);		
		fStrPwm  = calcStrpwm(odo,cvRate,cvOffset,odoEnd);//StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control4 done!! Next5:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),-lengthCenterToWaypoint,r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relHeading-M_PI),-M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp2定常円
			//CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),-M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp2定常円
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));
			Serial.print("len:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("cvRate:");Serial.print(cvRate);
			Serial.print("cvOffset:");Serial.print(cvOffset);Serial.print("odoEnd:");Serial.println(odoEnd);						
			odo = 0;
			controlMode = 5;
		}
	}
	else if(controlMode==5){//cp2定常円
		//yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = calcStrpwm(odo,cvRate,cvOffset,odoEnd);//StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control5 done!! Next6:,");//制御終了して次の制御へ
			GetLen(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),0,0,&l);
			GetPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,-r,&psi);
			CalcClothoidCurvatureRate(l,psi,relHeading,psi,&cvRate,&cvOffset,&odoEnd,5);//cp0へ
			//CalcClothoidCurvatureRate(l,psi,relYawAngle,psi,&cvRate,&cvOffset,&odoEnd,5);//cp0へ
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));
			Serial.print("len:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("cvRate:");Serial.print(cvRate);
			Serial.print("cvOffset:");Serial.print(cvOffset);Serial.print("odoEnd:");Serial.println(odoEnd);		
			odo = 0;
			controlMode = 6;
		}
	}
	else if(controlMode==6){//cp0へ
		//yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("relHeading:");Serial.println(relHeading);
		fStrPwm  = calcStrpwm(odo,cvRate,cvOffset,odoEnd);//StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control6 done!! Next1:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,-r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,relHeading,0,&cvRate,&cvOffset,&odoEnd,5);
			//CalcClothoidCurvatureRate(l,psi,relYawAngle,0,&cvRate,&cvOffset,&odoEnd,5);//cp1右側へ
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));
			Serial.print("len:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.print("cvRate:");Serial.print(cvRate);
			Serial.print("cvOffset:");Serial.print(cvOffset);Serial.print("odoEnd:");Serial.println(odoEnd);		
			odo = 0;
			controlMode = 1;
		}
		else if(f_Timer && odo > odoEnd-2){
			mode = 4;
		}
	}
	//Serial.print("odo:,");Serial.print(odo);Serial.print("TgtYaw:,");Serial.println(yawRt);
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
	if(rfromCenter < 30) {			//コース中央から半径20m内
		if(mode <= 0){
			Serial.print("yawAng, ");Serial.print(rpyAngle.z);Serial.print(",");Serial.print("yawRt, ");Serial.print(rpyRate.z);Serial.print(",");
			Serial.print("Heading, ");Serial.print(heading);Serial.print(",");
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
			Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));
			Serial.print("RelYaw:");Serial.print(relYawAngle);Serial.println(F(" ,To start, put key '1' , To learning direction, put Key '0'"));
			while(Serial.available()){
				switch(Serial.read()){
					case '0'	: mode = 1;imuCalibMode = 0;break; //再度IMU補正
					case '1'	: mode = 3;break; //走行開始
					default		: break;	
				}
			}
		}
		else if(mode == 3){
			if(Serial.available()){
				switch(Serial.read()){
					default		: mode = 4;break;	
				}
			}
		}
		else if(mode == 4){
			 Serial.print("RelYaw,");Serial.print(relYawAngle);Serial.print(",");
			 Serial.print("yawRt,");Serial.print(rpyRate.z);Serial.print(",");
			 Serial.print("x,");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print(",");
			 Serial.print("y,");Serial.print(rfromCenter * sin(bearfromCenter));Serial.print(",");		
		}
	}
	else{//コース外
		mode = -1;
		Serial.print("RelYaw,");Serial.print(relYawAngle);Serial.print(",");
		Serial.print("yawRt, ");Serial.print(rpyRate.z);Serial.print(",");
		Serial.print("Heading, ");Serial.print(heading);Serial.print(",");
		Serial.print("RelHeading, ");Serial.print(relHeading);Serial.print(",");
		Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print(",");
		Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));Serial.print(",");
		Serial.println("Out of Course");
	}
}


/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨー角フィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
 float StrControlPID(float currentYawAngle,float targetYawAngle)
 {
	float kP = 30,tI = 0.105,tD = 0.02625,bias = strPwmOffset;
	static float error_prior = 0,integral = 0;
	float error,derivative,output,sampleTime = 10 * 0.001;

	error = targetYawAngle - currentYawAngle;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = kP * (error + integral / tI + tD * derivative) + bias;

	error_prior = error;

	return output;
 }


/************************************************************************
 * FUNCTION : 速度コントロール
 * INPUT    :
 *
 * OUTPUT   : モータ制御指示値
 ***********************************************************************/
float SpdControlPID(float currentSpeed,float targetSpeed)
{
	float kP = 8,tI = 0.5,tD = 0.125,bias = 90;
	static float error_prior = 0,integral = 0;
	float error,derivative,output,sampleTime = 10 * 0.001;

	error = targetSpeed - currentSpeed;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = kP * (error + integral / tI + tD * derivative) + bias;
	output = LimitValue(output,80,180);
	error_prior = error;

	return output;
}

/************************************************************************
 * FUNCTION : クロソイド用変数取得
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
 void GetLen(float x0, float y0, float x1, float y1,float *len)
 {
	 *len = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
 }
 void GetPsi(float x0, float y0, float x1, float y1,float *psi)
{
	if(cos(relYawAngle)>0){
		*psi = atan2f((y1-y0),(x1-x0));
	}
	else{
		*psi = atan2f(-(y1-y0),-(x1-x0));
	}
}
void GetLenAndPsi(float x0, float y0, float x1, float y1,float *len, float *psi)
{
	GetLen(x0,y0,x1,y1,len);
	GetPsi(x0,y0,x1,y1,psi);
}
/************************************************************************
 * FUNCTION : クロソイド曲線用関数群
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
float Phi(float phi0, float phiV, float phiU, float S)
{
return phi0 + phiV * S + phiU * S * S;
}

Complex slope(float phi0,float phiV, float phiU, float S)
{
  Complex I(0,Phi(phi0, phiV, phiU, S));
  return I.c_exp();
}

int CalcParamClothoid(float phi0, float phi1 ,float phiV,float *psi,float *lambda ,int8_t n)
{
  Complex integral = (0,0),dA = (0,0); // 積分結果
  float w = 1/(float)n; // 積分範囲を n 個に分割したときの幅
  float S,phiUCalc;
  // === Simpson 法による積分 (開始） ===
  w = 1 / (float)n; // 分割幅
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

float CompMinPsi(float phi0, float phi1 ,float phiV , float targetpsi,float *lambda , int8_t n)
{
    float psi,x;
    CalcParamClothoid(phi0,phi1,phiV,&psi,lambda,n);
    Complex I1(0,psi),I2(0,(targetpsi - phi0));
    x = (I1.c_exp() - I2.c_exp()).modulus();
	  return x;
}

float CalcphiV(float (*f)(float,float,float,float,float *,int8_t), float *xmin, float phi0, float phi1 , float targetpsi, float *lambda, int8_t n)
{
	float f1, f2, x0, x1, x2, x3;
    float ax = -6.28;
    float bx = 6.28;
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

void CalcClothoidCurvatureRate(float l, float psi, float phi0,float phi1, float *cvRate,float *cvOffset,float *odoEnd,int n)
{
	float w; // 積分範囲を n 個に分割したときの幅
	float S,lambda,phiV,phiU,h,odo[n],cv[n];
	int i;
	// === Simpson 法による積分 (開始） ===
	w = 1.0 / (float)(n); // 分割幅
	S = 0;
	CalcphiV(CompMinPsi,&phiV,phi0,phi1,psi,&lambda,n);
	Serial.print("phiVCalc:");Serial.println(phiV);
	h = l / lambda;
	phiU = phi1 - phi0 - phiV;
	for (i=0; i<n; i++) {
		cv[i] = (phiV + 2 * phiU * S)/h;
		S += w;
		odo[i] = h*S;
	}
	*cvRate = (cv[n-1] - cv[0])/(odo[n-1]-odo[0]);
	*cvOffset = cv[0];
	*odoEnd = odo[n-1];
}

/************************************************************************
 * FUNCTION : クロソイド軌跡演算結果からヨーレート計算
 * INPUT    : 速度v,現在距離odo,曲率距離比cvRate,距離ゼロ時曲率
 * OUTPUT   : ヨーレート
 ***********************************************************************/
float calcYawRt(float v,float odo,float cvRate,float cvOffset,float odoend)
{
    float yawRt;
    if(odo >= 0 && odo < odoend){yawRt = v * ((odo * cvRate) + cvOffset);}
    else if(odo >= odoend){yawRt = v * ((odoend * cvRate) + cvOffset);}
    else{yawRt = v * cvOffset;}
    return yawRt;
}

/************************************************************************
 * FUNCTION : クロソイド軌跡演算結果から操舵PWM指示値計算
 * INPUT    : 速度v,現在距離odo,曲率距離比cvRate,距離ゼロ時曲率
 * OUTPUT   : 操舵PWM指示値
 ***********************************************************************/
float calcStrpwm(float odo,float cvRate,float cvOffset,float odoend)
{
	float pwm;
    if(odo >= 0 && odo < odoend){pwm = kStrAngle2Pwm * (wheelbase * ((odo * cvRate) + cvOffset)) + strPwmOffset;}
    else if(odo >= odoend){pwm = kStrAngle2Pwm * (wheelbase * ((odoend * cvRate) + cvOffset)) + strPwmOffset;}
    else{pwm = strPwmOffset;}
    return pwm;
}

/************************************************************************
 * FUNCTION : IMU加速度生値->物理値変換
 * INPUT    : IMU加速度生値
 * OUTPUT   : 加速度(m/s^2)
 ***********************************************************************/
float convertRawAcceleration(int aRaw)
{
   float a = (aRaw * 2.0 * 9.80665) / 32768.0;
   return a;
}

/************************************************************************
 * FUNCTION : IMU角速度生値->物理値変換
 * INPUT    : IMU角速度生値
 * OUTPUT   : 角速度(deg/s)
 ***********************************************************************/
float convertRawGyro(int gRaw)
{
   float g = (gRaw * 500.0f) / 32768.0;
   return g;
}

/************************************************************************
 * FUNCTION : IMUによるGPS方位情報補間
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
void HeadingUpdateIMU(float *heading,float *relHeading, float yawRtIMU,float SampleTime10ms)
{
	*heading += yawRtIMU * SampleTime10ms;
	*relHeading += yawRtIMU * SampleTime10ms;
}

/************************************************************************
 * FUNCTION : 指定秒タイマ
 * INPUT    : タイマ時間(s)
 * OUTPUT   : タイマフラグ 
 ***********************************************************************/
uint8_t TimerSec(unsigned int timerTime)
{
	static unsigned int startTime = millis() * 0.001;
	unsigned int  timeNow = millis() * 0.001;
	int countDownTime = timerTime - (timeNow - startTime);
	if(countDownTime < 0){
		return 1;
	}
	else{
		Serial.print("Timer : ");Serial.print(countDownTime);
		return 0;
	}
}

/************************************************************************
 * FUNCTION : 10msサンプリングタイム取得
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
void GetSampleTime10ms(float *SampleTime10ms)
{
	static unsigned long lasttime = 0;
	*SampleTime10ms = (millis() - lasttime) * 0.001;
	lasttime = millis();
}


/************************************************************************
 * FUNCTION : 角度を-piからpiの範囲に納める
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
float Pi2pi(float angle)
{
    while(angle >= M_PI) {angle -= M_PI * 2;}
    while(angle < -M_PI){angle += M_PI * 2;}
    return angle;
}

/************************************************************************
 * FUNCTION : メインループ関数
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
void loop()
{
	runner.execute();
}

/************************************************************************
 * FUNCTION : シリアルポート1受信割り込み処理
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
void serialEvent1(){
	bool gpsReadDone = false;
	while (gps.available(Serial1)) {
		fix = gps.read();
		gpsReadDone = true;
	}
	if(gpsReadDone){
		GPSFIX();
	}
}
