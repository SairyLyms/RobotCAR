/************************************************************************/
/*	Include Files														*/
/************************************************************************/
#include <NMEAGPS.h>
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
int8_t n = 5;
float lengthCenterToWaypoint = 7;
float r = 3;
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
NeoGPS::Location_t cp1(365680460L,1399958340L);
NeoGPS::Location_t cp2(365679580L,1399958340L);
#elif defined Home
NeoGPS::Location_t cp1(385071520L,1403969840L);
NeoGPS::Location_t cp2(385071540L,1403970570L);
#elif defined Ground
NeoGPS::Location_t cp1(385156170L,1403968200L);
NeoGPS::Location_t cp2(385155440L,1403966060L);
#endif
NeoGPS::Location_t locCenter;

volatile NMEAGPS  gps;
volatile gps_fix  fix;
MPU6050 IMU;
Servo FStr,PowUnit;
Scheduler runner;
/*座標系は安部先生の本に従う(右手系)*/
volatile VectorFloat rpyAngle;
volatile bool update_data;
volatile float relYawAngle = 0;												/*CPまでの角度(Heading:CCW+)*/

VectorFloat rpyRate;
VectorFloat acc;
volatile float rfromCenter,bearfromCenter;
volatile float heading;											/*方位(rad,deg)*/
volatile float gpsSpeedmps;										/*GPS絶対速度(m/s)*/
float directioncp;
volatile float relHeading;							 					/*CP間中心線からの相対角度(Heading:CCW+)*/
float headingOffstIMU;
float puPwm = 90;												/*パワーユニットのPWM*/
float fStrPwm = 90;												/*ステアリングのPWM*/
uint8_t sampleTimems = 10;
int8_t mode,imuCalibMode;
uint16_t packetSize;
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
	IMU.dmpInitialize();
	
	IMU.setSleepEnabled(false);
	IMU.setDMPEnabled(true);
	IMU.setXAccelOffset(454);IMU.setYAccelOffset(-3773);IMU.setZAccelOffset(630);
	IMU.setXGyroOffset(107);IMU.setYGyroOffset(-23);IMU.setZGyroOffset(12);

	packetSize = IMU.dmpGetFIFOPacketSize();

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
 * FUNCTION : GPS_Fix後更新処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
 void GPSFIX(void)
 {
	 if(fix.valid.speed){gpsSpeedmps = fix.speed() * 0.514444;}
	 if(fix.valid.heading){heading = -fix.heading() * 0.01745329251;relHeading = Pi2pi(heading + directioncp);}
	 if(fix.valid.location) {
		NeoGPS::Location_t current(fix.location.lat(),fix.location.lon());
		rfromCenter = fix.location.DistanceKm(locCenter) * 1000.0f;
		bearfromCenter = -Pi2pi(locCenter.BearingTo(current) - directioncp);
	}
	 //Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.println(rfromCenter * sin(bearfromCenter));
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

float ave5(float in)//5回平均計算
{
	static float buf[5];
	float ave;
	for(int8_t i=0;i<5;i++){
		i > 0 ? buf[i] = buf[i-1] : buf[0] = in;
		ave += buf[i];
	}
	ave *= 0.2;
	return ave;
}

float ave50(float in)//50回平均計算(50回計算後に初めて出力)
{
	static float buf[50];
	float ave;
		for(int8_t i=0;i<50;i++){
			i > 0 ? buf[i] = buf[i-1] : buf[0] = in;
			ave += buf[i];
		}
	buf[49] != 0 ? ave *= 0.02 : ave = 0;
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
	int gix, giy, giz;

	IMU.getMotion6(NULL, NULL, NULL, &gix, &giy, &giz);

	gfx = convertRawGyro(giy);
	gfy = -convertRawGyro(gix);
	gfz = convertRawGyro(giz);

	rpyRate.x = gfx * M_PI/180;
	rpyRate.y = gfy * M_PI/180;
	rpyRate.z = gfz * M_PI/180;

#ifdef DEBUG_IMU
	Serial.print("Time, ");
	Serial.print(",");
	Serial.print(millis() * 0.001);
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

void DMPupdate(float *_roll, float *_pitch, float *_yaw, bool *update_data)
{
	
  // 戻り値のリセット
  *update_data = false;
  // センサーの状態を取得(INT_STATUS レジスタ)
  const uint8_t mpuIntStatus = IMU.getIntStatus();
  if (mpuIntStatus & 0x12)
  {
    // FIFOのデータサイズを取得
    const uint16_t fifoCount = IMU.getFIFOCount();
    if ((mpuIntStatus & 0x10) || (1024 <= fifoCount))
    {
      // オーバーフローを検出したらFIFOをリセット(そもそも検出されないコトあり…)
      IMU.resetFIFO();
      Serial.println(F("FIFO overflow!"));
    }
    else if((mpuIntStatus & 0x02) && (packetSize <= fifoCount))
    {
      // データの読み出し可能
      *update_data = true;
    }
 
    // FIFOよりデータを読み出す
    if (*update_data)
    {
      uint8_t fifoBuffer[64]; // FIFO storage buffer
      IMU.getFIFOBytes(fifoBuffer, packetSize);
 
      // データをオイラー角へ変換
      Quaternion q;           // [w, x, y, z]         quaternion container
      VectorFloat gravity;    // [x, y, z]            gravity vector
	  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	  
      IMU.dmpGetQuaternion(&q, fifoBuffer);
      IMU.dmpGetGravity(&gravity, &q);
      IMU.dmpGetYawPitchRoll(ypr, &q, &gravity);
	  *_yaw   = -ypr[0];
      *_pitch = ypr[1];
	  *_roll  = ypr[2];
	  relYawAngle = Pi2pi(*_yaw + headingOffstIMU);
      IMU.resetFIFO();
    }
  }
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
	case 0:		fStrPwm = StrControlPID(rpyRate.z,0.0);	
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
	case 0xf:	puPwm = SpdControlPID(gpsSpeedmps,2.0);
				fStrPwm = StrControlPID(rpyRate.z,0.0);
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
	static double cvRate,cvOffset,odoEnd,odo;
	double yawRt;
	float l,psi;
	//Serial.print("Current Mode :");Serial.print(controlMode);
	//Serial.print("psi:,");Serial.print(psi);
	if(controlMode==0){
		Serial.print("RelYaw:");Serial.println(relYawAngle);
		GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,-r,&l,&psi);
		Serial.print("L:");Serial.print(l);Serial.print("psi:");Serial.print(psi);
		Serial.print("RelYaw:");Serial.println(relYawAngle);
		CalcClothoidCurvatureRate(l,psi,relYawAngle,0,&cvRate,&cvOffset,&odoEnd,5);
		controlMode = 1;
	}
	else if(controlMode==1){//初回cp1旋回開始
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		Serial.print("odoEnd:");Serial.print(odoEnd);Serial.print("cvOfst:");Serial.print(cvOffset);
		Serial.print("cvRate:");Serial.print(cvRate);Serial.print("Spd:");Serial.print(gpsSpeedmps);
		Serial.print("TgtYawRt:");Serial.print(yawRt);Serial.print("RelYaw:");Serial.print(relYawAngle);
		Serial.print("YawRt:");Serial.println(rpyRate.z);
		fStrPwm  = StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;
		if(odo>odoEnd){
			Serial.print("RelYaw:");Serial.println(relYawAngle);
			Serial.println("Control1 done!!:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,relYawAngle,M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp1反対側へ定常円			
			odo = 0;
			controlMode = 2;
		}
	}
	else if(controlMode==2){//cp1定常円
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.print("RelYaw:");Serial.println(relYawAngle);
			Serial.println("Control2 done!!:,");//制御終了して次の制御へ
			//GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),0,0,&l,&psi);
			GetLen(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),0,0,&l);
			GetPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),-lengthCenterToWaypoint,-r,&psi);
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),psi,&cvRate,&cvOffset,&odoEnd,5);//cp0へ						
			odo = 0;
			controlMode = 3;
		}
	}
	else if(controlMode==3){//cp0へ
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control3 done!!:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),-lengthCenterToWaypoint,-r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),0,&cvRate,&cvOffset,&odoEnd,5);//cp2へ						
			odo = 0;
			controlMode = 4;
			}
	}
	else if(controlMode==4){//cp2へ
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control4 done!!:,");//制御終了して次の制御へ
			GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),-lengthCenterToWaypoint,r,&l,&psi);
			CalcClothoidCurvatureRate(l,psi,Pi2pi(relYawAngle-M_PI),-M_PI,&cvRate,&cvOffset,&odoEnd,5);//cp2定常円					
			odo = 0;
			controlMode = 5;
		}
	}
	else if(controlMode==5){//cp2定常円
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
			Serial.println("Control5 done!!:,");//制御終了して次の制御へ
			//GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),0,0,&l,&psi);
			GetLen(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),0,0,&l);
			GetPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,-r,&psi);
			CalcClothoidCurvatureRate(l,psi,relYawAngle,psi,&cvRate,&cvOffset,&odoEnd,5);//cp0へ
			odo = 0;
			controlMode = 6;
		}
	}
	else if(controlMode==6){//cp0へ
		yawRt = calcYawRt(gpsSpeedmps,odo,cvRate,cvOffset,odoEnd);
		fStrPwm  = StrControlPID(rpyRate.z,yawRt);
		odo += gpsSpeedmps * sampleTimems * 0.001;		
		if(odo>odoEnd){
				Serial.println("Control6 done!!:,");//制御終了して次の制御へ
				GetLenAndPsi(rfromCenter * cos(bearfromCenter),rfromCenter * sin(bearfromCenter),lengthCenterToWaypoint,-r,&l,&psi);
				CalcClothoidCurvatureRate(l,psi,relYawAngle,0,&cvRate,&cvOffset,&odoEnd,5);//cp1右側へ
				odo = 0;
				controlMode = 1;
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
	if(rfromCenter < 20) {			//コース中央から半径20m内
		if(mode <= 0){
			Serial.print("yawAng, ");Serial.print(rpyAngle.z);
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
			//Serial.println("Now Mode 3");
		}
	}
	else{//コース外
		mode = -1;
		Serial.print("yawRt, ");Serial.print(rpyRate.z);Serial.print("yawAng, ");Serial.print(rpyAngle.z);
		Serial.print("x:");Serial.print(rfromCenter * cos(bearfromCenter));Serial.print("y:");Serial.print(rfromCenter * sin(bearfromCenter));		
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
 * FUNCTION : 操舵制御指示値演算(ヨー角フィードバック)
 * INPUT    : CPまでの目標角度、ヨーレート、
 *            強制操舵方向指定(0:通常操舵、1:左旋回、-1:右旋回)
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
 float StrControlPIDAng(int8_t mode, float value, float yawAngle,float targetYawAngle)
 {
	float sampleTime = sampleTimems * 0.001;
 #ifdef CC01
  float kp = 180,ti = 0.5 * 0.42 ,td = 0.125 * 0.42,diff;
 #else
 	float kp = 2.125,ti = 0.4 ,td = 0.1,diff;
 #endif
 	static float err[3],lastyawAngle,lastvalue;
 	static int8_t lastMode;

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
float SpdControlPID(float currentSpeed,float targetSpeed)
{
	float KP = 8,TI = 0.5,TD = 0.125,bias = 90;
	static float error_prior = 0,integral = 0;
	float error,derivative,output,sampleTime = 10 * 0.001;

	error = targetSpeed - currentSpeed;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = KP * (error + integral / TI + TD * derivative) + bias;
	//Serial.print("outputraw:");Serial.println(output);
	output = LimitValue(output,80,180);
	error_prior = error;
	//Serial.print("Time:");Serial.print(millis());Serial.print("Speed:");Serial.print(gpsSpeedmps);
	//Serial.print("error:");Serial.print(error);Serial.print("output:");Serial.println(output);

	return output;
}

/************************************************************************
 * FUNCTION : コース設定
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

float convertRawGyro(int gRaw)
 {
	// since we are using 250 degrees/seconds range
	// -250 maps to a raw value of -32768
	// +250 maps to a raw value of 32767

	float g = (gRaw * 2000.0f) / 32768.0;
	return g;
}


float Pi2pi(float angle)
{
    while(angle >= M_PI) {angle -= M_PI * 2;}
    while(angle < -M_PI){angle += M_PI * 2;}
    return angle;
}

void loop()
{
	runner.execute();
}

void serialEvent1(){
	bool gpsReadDone = false;
	while (gps.available(Serial1)) {
		fix = gps.read();
		gpsReadDone = true;
	}
	if(gpsReadDone){
	GPSFIX();
	DMPupdate(&rpyAngle.x,&rpyAngle.y,&rpyAngle.z,&update_data);
	}
}
