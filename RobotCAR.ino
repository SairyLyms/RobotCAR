/************************************************************************/
/*	Include Files														*/
/************************************************************************/
#include <NMEAGPS.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <TaskScheduler.h>
#include <Filters.h>
#include "Param.h"
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

//#define CC01
//#define DEBUG_IMU
//#define DEBUG_GPS
//#define DEBUG
//#define TechCom											/*�e�X�g�R�[�X�ݒ�*/
//#define Ground
#define Garden
//#define HappiTow

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
HMC5883L MAG;
Madgwick AHRS;
Servo FStr,PowUnit;
Scheduler runner;
/*���W�n�͈����搶�̖{�ɏ]��(�E��n)*/
VectorFloat rpyAngle;
VectorFloat rpyRate;
VectorFloat acc;                                                  /*X,Y,Z�����x(m/s^2)*/
float relHeading;							 																		/*CP�Ԓ��S������̑��Ίp�x(Heading:CW+,Yaw:CCW+)*/
float relHeadingTgt[2];																						/*CP�܂ł̊p�x(Heading:CW+)*/
float heading, headingDeg;																				/*����(rad,deg)*/
float yawRtGPS;
float headingOffst = cp[1].BearingTo(cp[0]);											/*CP�Ԃ̕���*/
float gpsSpeedmps;                                               /*GPS��Α��x(m/s)*/
polVectorFloat3D centerPos;
uint8_t sats;
int puPwm = 90;																									/*�p���[���j�b�g��PWM*/
int fStrPwm = 90;                                               /*�X�e�A�����O��PWM*/
float fStrDeg;																									/*�O�֎��Ǌp*/

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
	IMU.initialize();
	IMU.setDLPFMode(MPU6050_DLPF_BW_10);
	IMU.setXGyroOffset(50);
	IMU.setYGyroOffset(-35);
	IMU.setZGyroOffset(20);
	IMU.setXAccelOffset(-18);
	IMU.setYAccelOffset(-82);
	IMU.setZAccelOffset(2170);
	FStr.attach(3);
	PowUnit.attach(2,1000,2000);
	Serial.flush();
	runner.startNow();
	Initwait();
}

/************************************************************************
 * FUNCTION : 10 ms��������
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
void Task10ms(void)
{
	IMUupdate();
	IntegratedChassisControl();
	DisplayInfo();
}

/************************************************************************
 * FUNCTION : 20 ms��������
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
void Task20ms(void)
{
	ChassisFinalOutput(puPwm,fStrPwm);
}

/************************************************************************
 * FUNCTION : 100 ms��������
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
void Task100ms(void)
{
}

/************************************************************************
 * FUNCTION : 1000 ms��������
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
void Task1000ms(void)
{
}

/************************************************************************
 * FUNCTION : GPS�X�V����
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
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
			distToCP[i].t = fix.location.BearingTo(cpAway[i]); //���i������p�̃��t�@�����X����
			distToCP[i].p = fix.location.BearingTo(cp[i]); //����J�n����p�̕���
			}
			centerPos.r = locCenter.DistanceKm(fix.location) * 1000.0f;
			centerPos.t = locCenter.BearingTo(fix.location);
		}
	fix.valid.speed ? gpsSpeedmps = fix.speed() * 0.514444 : 0;
	fix.valid.satellites ? sats = fix.satellites : 0;
	if(fix.valid.heading){
		heading ? yawRtGPS = -CalcRPYRate(heading,fix.heading() * 0.01745329251,sampleTime) : 0;
		heading = fix.heading() * 0.01745329251;
		heading && headingOffst ? relHeading = RoundRadPosNeg(heading - headingOffst) : 0;
		for(int8_t i=0;i<2;i++){
			distToCP[i].t ? relHeadingTgt[i] = RoundRadPosNeg(heading-distToCP[i].t) : 0;
			}
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
 * FUNCTION : IMU�X�V����
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
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

	IMU.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

	afx = convertRawAcceleration(aiy);
	afy = -convertRawAcceleration(aix);
	afz = convertRawAcceleration(aiz);
	gfx = convertRawGyro(giy);
	gfy = -convertRawGyro(gix);
	gfz = convertRawGyro(giz);

	AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);

#if 0
	rpyAngle.x ? rpyRate.x = CalcRPYRate(rpyAngle.x,AHRS.getRoll() * M_PI/180,sampleTime) : 0;
	rpyAngle.y ? rpyRate.y = CalcRPYRate(rpyAngle.y,AHRS.getPitch() * M_PI/180,sampleTime) : 0;
	rpyAngle.z ? rpyRate.z = CalcRPYRate(rpyAngle.z,AHRS.getYaw() * M_PI/180,sampleTime) : 0;
#endif

	rpyRate.x = gfx * M_PI/180;
	rpyRate.y = gfy * M_PI/180;
	rpyRate.z = gfz * M_PI/180;

	rpyAngle.x = AHRS.getRoll() * M_PI/180; //���[���p
	rpyAngle.y = AHRS.getPitch() * M_PI/180; //�s�b�`�p
	rpyAngle.z = AHRS.getYaw() * M_PI/180; //���[�p

	acc.x = convertRawAcceleration(aiy + (16384 * sin(rpyAngle.y) * cos(rpyAngle.x))); //�d�͂̉e�������O���������xx
	acc.y = convertRawAcceleration(-aix - (16384 * cos(rpyAngle.y) * sin(rpyAngle.x))); //�d�͂̉e�������O���������xy
	acc.z = convertRawAcceleration(aiz - (16384 * cos(rpyAngle.y) * cos(rpyAngle.x))); //�d�͂̉e�������O���������xz


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

/************************************************************************
 * FUNCTION : �R���p�X�X�V����
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
 void MAGupdate(void)
 {
	 int16_t mx, my, mz;
	 MAG.getHeading(&mx, &my, &mz);

	 float heading = atan2(my, mx);
	 if(heading < 0)
		 heading += 2 * M_PI;

	 Serial.print("mag:\t");
	 Serial.print(mx); Serial.print("\t");
	 Serial.print(my); Serial.print("\t");
	 Serial.print(mz); Serial.print("\t");

	 Serial.print("heading:\t");
	 Serial.println(heading * 180/M_PI);
 }


/************************************************************************
 * FUNCTION : IMU�X�V����
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
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
 * FUNCTION : �T�[�{�o��(�e�X�g��)
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
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
 * FUNCTION : �V���V��������(�e�X�g��)
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
void IntegratedChassisControl(void)
{
	static float targetAngleCP,targetYawRtCP;
	int8_t key;
	static int8_t select;
	VectorFloat targetpoint;
	key = Serial.read();
	switch (key) {
	case 'a': select = 1;break;

	case 'd': select = 2;break;

	case '0':
					if(select == 1){
						puPwm = 90;
					}
					else if(select == 2){
						fStrDeg = 0;
						fStrPwm = (int)(Lookup1D(fStrDeg,fStrDegx,kfStrDeg2fPWM,sizeof(fStrDegx)/sizeof(fStrDegx[0]))+90);
					}
					else{
						fStrPwm = 0;
						puPwm = 90;
					}
					break;

	case '1':
				if(select == 1){
					puPwm+= 5;
					//Serial.print("PuPwm:,"); Serial.println(puPwm);
				}
				else if(select == 2){
					fStrDeg+=2;
					fStrPwm = (int)(Lookup1D(fStrDeg,fStrDegx,kfStrDeg2fPWM,sizeof(fStrDegx)/sizeof(fStrDegx[0]))+90);
					//Serial.print("StrDeg:,"); Serial.print(deg);Serial.print("\t");Serial.print("StrPwm:,"); Serial.println(fStrPwm);
				}
				else{
				}
	break;

	case '2':
				if(select == 1){
					puPwm-= 5;
					//Serial.print("PuPwm:,"); Serial.println(puPwm);
				}
				else if(select == 2){
					fStrDeg-=2;
					fStrPwm = (int)(Lookup1D(fStrDeg,fStrDegx,kfStrDeg2fPWM,sizeof(fStrDegx)/sizeof(fStrDegx[0]))+90);
					//Serial.print("StrDeg:,"); Serial.print(deg);Serial.print("\t");Serial.print("StrPwm:,"); Serial.println(fStrPwm);
				}
				else{
				}
	break;
	default:
	break;
	}
	#if 1
	#ifdef DEBUG
	//Serial.print(",TgtAngle:,"); Serial.print(targetAngleCP);
	//Serial.print(",TgtYawRt:,"); Serial.print(targetYawRtCP);Serial.print(",yawRate:,"); Serial.print(rpyRate.z,6); Serial.print(",Speed:,"); Serial.print(gpsSpeedmps);
	//Serial.print(",Mode:,"); Serial.print(mode);Serial.print("StrPWM:,"); Serial.println(fStrPwm);
	#endif
	#endif
}


/************************************************************************
 * FUNCTION : ��ԊǗ�
 * INPUT    :
 *
 * OUTPUT   : ��Ԓl
 ***********************************************************************/
int8_t StateManager(NeoGPS::Location_t cp[],polVectorFloat3D distToCP[], polVectorFloat3D centerPos,float relHeading)
{
	static int8_t mode;

	return mode;
}

/************************************************************************
 * FUNCTION : �ڕW���[���[�g���Z
 * INPUT    : �Ȃ�
 * OUTPUT   : �Ȃ�
 ***********************************************************************/
float CalcTargetYawRt(polVectorFloat3D distToCP,float relHeadingTgt,float gpsSpeedmps)
{
	float TargetYawRt;
	TargetYawRt = gpsSpeedmps * sin(relHeadingTgt)/distToCP.r;			/*�{���̓R�[�X�ɑ΂��郈�[�p�̕΍�����͂��ׂ��������׍팸�̂���relHeadingTgt�ő�p*/
#if DEBUG
	Serial.print(",relYaw:,");Serial.print(relHeadingTgt);
	Serial.print(",distToCP:,"); Serial.print(distToCP.r);
	Serial.print(",tgtYawRt:,"); Serial.println(TargetYawRt);
	return TargetYawRt;
#endif
}

/************************************************************************
 * FUNCTION : ���ǐ���w���l���Z(���[���[�g�t�B�[�h�o�b�N)
 * INPUT    : CP�܂ł̖ڕW�p�x�A���[���[�g�A
 *            �������Ǖ����w��(0:�ʏ푀�ǁA1:������A-1:�E����)
 * OUTPUT   : ���ǐ���w���l(�T�[�{�p)
 ***********************************************************************/
float StrControlPID(int8_t mode, float value, VectorFloat rpyRate,float targetYawRt)
{
	float sampleTime;
#ifdef CC01
	float kp = 0.5,ti = 0.1 ,td = 0.25,diff;
#else
	float kp = 2.125,ti = 0.4 ,td = 0.1,diff;
#endif
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
 * FUNCTION : ���x�R���g���[��
 * INPUT    :
 *
 * OUTPUT   : ���ǐ���w���l(�T�[�{�p)
 ***********************************************************************/
float SpdControlPID(void)
{
}

/************************************************************************
 * FUNCTION : 1D�e�[�u�����b�N�A�b�v�֐�
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
 float Lookup1D(float value,float tableX[],float tableK[],int8_t tableSize)
  {
 	 float buf;
 	 int sign = (value > 0) - (value < 0);
 	 if(tableSize > 2){
 	 for(int8_t i=0;i<tableSize;i++){
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

float RoundRad(float x) //�p�x��\���ϐ���0�`6.28rad(2 * M_PI)�͈̔͂Ɏ��߂�֐�
{
	if (x >= 0.f) {
		return fmod(x, 2 * M_PI);
	} else {
		return 2 * M_PI - fmod(-x, 2 * M_PI);
	}
	return x;
}

float RoundRadPosNeg(float x)//�p�x��\���ϐ���-3.14�`3.14rad(M_PI)�͈̔͂Ɏ��߂�֐�
{
    return fmod(x + M_PI, 2 * M_PI) - M_PI;
}

float RoundDeg(float x) //�p�x��\���ϐ���0�`360deg�͈̔͂Ɏ��߂�֐�
{
	if (x >= 0.f) {
		return fmod(x, 360.f);
	} else {
		return 360.f - fmod(-x, 360.f);
	}
	return x;
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

void DisplayInfo(void)
{
	Serial.print("StrDeg: ");
	Serial.print(fStrDeg);
	Serial.print(",");
	Serial.print("PuPWM: ");
	Serial.print(puPwm);
	Serial.print(",");
	Serial.print("Ax: ");
	Serial.print(acc.x);
	Serial.print(",");
	Serial.print("Ay: ");
	Serial.print(acc.y);
	Serial.print(",");
	Serial.print("yawRt: ");
	Serial.print(rpyRate.z);
	Serial.print(",");
	Serial.print("YawRtGPS: ");
	Serial.print(yawRtGPS);
	Serial.print(",");
	Serial.print("Speed:");
	Serial.print(gpsSpeedmps);
	Serial.print(",");
	Serial.println(",");
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
