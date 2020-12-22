#include <MsTimer2.h>

#define PIN_PWMR 9
#define ENCODER_AR 3//引发中断
#define ENCODER_BR 5

#define PIN_PWML 10
#define ENCODER_AL 2//引发中断
#define ENCODER_BL 4

#define MAX_Motor_Vol 254
#define MAX_deltaVoltL 128//最大左轮修正电压
#define MAX_deltaVoltR 128//最大右轮修正电压

#define GET_SPEED_PERIOD_MS 20//获取速度周期（单位毫秒）

double SpeedP = 0.2; //电机调速比例项
double SpeedD = 0.03;//电机调速微分项

enum MOTOR_SYNC_MODE {SyncOff, SyncOn} motorSyncMode;//左右电动机同步模式

volatile long encoderValR = 0;
volatile long encoderValL = 0;

volatile long lastEncoderValR = 0;
volatile long lastEncoderValL = 0;

int speedR = 0;//实际右轮速度
int speedL = 0;//实际左轮速度

double expSpeedR = 0;//期望右轮速度
double expSpeedL = 0;//期望左轮速度

//loop函数内相关
unsigned long lastT = 0;//上次loop微秒值
unsigned long dt = 0;//时间微分，单位微秒


double voltL = 0;
double voltR = 0;
double preVoltL = 127;
double preVoltR = 127;
double deltaVoltPL = 0;//正转时左轮修正电压
double deltaVoltPR = 0;//正转时右轮修正电压
double deltaVoltNL = 0;//反转时左轮修正电压
double deltaVoltNR = 0;//反转时右轮修正电压

void setup()
{
	pinMode(PIN_PWMR, OUTPUT);
	pinMode(PIN_PWML, OUTPUT);

	analogWrite(PIN_PWMR,127);
	analogWrite(PIN_PWML,127);

	pinMode(ENCODER_AR, INPUT);
	pinMode(ENCODER_BR, INPUT);
	pinMode(ENCODER_AL, INPUT);
	pinMode(ENCODER_BL, INPUT);
	// pinMode(IR1, INPUT);
	// pinMode(IR2, INPUT);
	// pinMode(IR3, INPUT);
	// pinMode(IR4, INPUT);
	// pinMode(IR5, INPUT);
	// pinMode(IR6, INPUT);
	// pinMode(IR7, INPUT);
	// pinMode(IR8, INPUT);
	// pinMode(IR9, INPUT);

	Serial.begin(115200);

	attachInterrupt(digitalPinToInterrupt(ENCODER_AR), getEncoderR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_AL), getEncoderL, CHANGE);

	motorSyncMode = SyncOn;

	MsTimer2::set(GET_SPEED_PERIOD_MS, GetSpeed);
	MsTimer2::start();

	TCCR1B = TCCR1B & 0xF8 | 1;//控制pin9 pin10频率，可以是 1, 2, 3, 4, 5
	/* 其中 fff 與對應頻率如下:
	feqT1		Prescaler	Frequency
	1		1			31372.549 Hz
	2		8			3921.569
	3		64			490.196   <--DEFAULT
	4		256			122.549
	5		1024		30.637 Hz */

	//TCCR2B = TCCR2B & 0xF8 | 3;//控制pin11 pin3频率，可以是 1, 2, 3, 4, 5, 6, 7（不要调，会影响MsTimer2)
/* 
	feqT2	Prescaler	Frequency
	1		1			31372.549 Hz
	2		8			3921.569
	3		32			980.392
	4		64			490.196   <--DEFAULT
	5		128			245.098
	6		256			122.549
	7		1024		30.637 Hz
 */
}

void getEncoderR(void) 
{
	if (digitalRead(ENCODER_AR) == LOW)
	{
		if (digitalRead(ENCODER_BR) == LOW) 
		{
			encoderValR--;
		}
		else
		{
			encoderValR++;
		}
	}
	else
	{
		if (digitalRead(ENCODER_BR) == HIGH)
		{
			encoderValR--;
		}
		else
		{
			encoderValR++;
		}
	}
}

void getEncoderL(void) 
{
	if (digitalRead(ENCODER_AL) == LOW)
	{
		if (digitalRead(ENCODER_BL) == LOW)
		{
			encoderValL--;
		}
		else
		{
			encoderValL++;
		}
	}
	else
	{
		if (digitalRead(ENCODER_BL) == HIGH)
		{
			encoderValL--;
		}
		else
		{
			encoderValL++;
		}
	}
	
	
}

//建议 turnRatio 范围[-1, 1]，不会自动切换模式
void TurnLeft(double speed, double turnRatio)
{
	//motorSyncMode = SyncOn;
	expSpeedR = speed * (1 - turnRatio);
	expSpeedL = speed * (1 + turnRatio);

/* 	if (expSpeedL > 0)
	{
		//preVoltL = 0.0175 * expSpeedL + 127.54;
	}
	else
	{
		//preVoltL = 0.0165 * expSpeedL - 129.96;
	}
	
	if (expSpeedR > 0)
	{
		//preVoltR = 0.0177 * expSpeedR + 131.12;
	}
	else
	{
		//preVoltR = 0.0184 * expSpeedR - 132.95;
	} */
}

void MotorSync()
{
	static double speedLErr = 0;
	static double last_speedLErr = 0;
	static double speedRErr = 0;
	static double last_speedRErr = 0;
	switch (motorSyncMode)
	{
	case SyncOn:
		//左轮
		last_speedLErr = speedLErr;
		speedLErr = expSpeedL - speedL;

		deltaVoltPL += speedLErr * SpeedP * dt / 1000000 + (speedLErr - last_speedLErr) * SpeedD;
		voltL = preVoltL + deltaVoltPL;

		//限制deltaVolt
		if (deltaVoltPL > MAX_deltaVoltL){
		deltaVoltPL = MAX_deltaVoltL;
		}else if (deltaVoltPL < -MAX_deltaVoltL){
		deltaVoltPL = -MAX_deltaVoltL;}

		//右轮
		last_speedRErr = speedRErr;
		speedRErr = expSpeedR - speedR;
		deltaVoltPR += speedRErr * SpeedP * dt / 1000000 + (speedRErr - last_speedRErr) * SpeedD;
		voltR = preVoltR + deltaVoltPR;

		//限制deltaVolt
		if (deltaVoltPR > MAX_deltaVoltR){
		deltaVoltPR = MAX_deltaVoltR;
		}else if (deltaVoltPR < -MAX_deltaVoltR){
		deltaVoltPR = -MAX_deltaVoltR;}

		//限制volt
		if (voltL > MAX_Motor_Vol){
			voltL = MAX_Motor_Vol;
		}else if (voltL < 0){
			voltL = 0;}

		if (voltR > MAX_Motor_Vol){
			voltR = MAX_Motor_Vol;
		}else if (voltR < -MAX_Motor_Vol){
			voltR = -MAX_Motor_Vol;}

		analogWrite(PIN_PWMR, voltR / 2 + 127);
		analogWrite(PIN_PWML, voltL / 2 + 127);

		break;
	case SyncOff:
		analogWrite(PIN_PWMR, voltR / 2 + 127);
		analogWrite(PIN_PWML, voltL / 2 + 127);

		break;
	default:
		break;
	}
}

void PrintData()
{
	Serial.print("\nSL ");
	Serial.print(speedL / 10);
	Serial.print(",SR ");
	Serial.print(speedR / 10);
	Serial.print(",ExpL ");
	Serial.print(expSpeedL / 10);
	Serial.print(",ExpR ");
	Serial.print(expSpeedR / 10);
	Serial.print(",VoltL ");
	Serial.print(voltL);
	Serial.print(",VoltR ");
	Serial.print(voltR);

	// Serial.print(",dVPL ");
	// Serial.print(deltaVoltPL);
	// Serial.print(",dVPR ");
	// Serial.print(deltaVoltPR);
	// Serial.print(",dVNL ");
	// Serial.print(deltaVoltNL);
	// Serial.print(",dVNR ");
	// Serial.print(deltaVoltNL);
	
	Serial.print(" dt= ");
	Serial.print(dt / 100);
	// Serial.print(digitalRead(IR1));
	// Serial.print(digitalRead(IR2));
	// Serial.print(digitalRead(IR3));
	// Serial.print(digitalRead(IR4));
	// Serial.print(digitalRead(IR5));
	// Serial.print(digitalRead(IR6));
	// Serial.print(digitalRead(IR7));
	// Serial.print(digitalRead(IR8));
	// Serial.print(digitalRead(IR9));
	// Serial.print('\n');
	
}

void GetSpeed()
{
	speedR = (encoderValR - lastEncoderValR) * 1000 / GET_SPEED_PERIOD_MS;
	speedL = (encoderValL - lastEncoderValL) * 1000 / GET_SPEED_PERIOD_MS;

	lastEncoderValR = encoderValR;
	lastEncoderValL = encoderValL;
	
}

//返回限制在[-MAX_value, MAX_value]的数
double Limit(double value, double MAX_value)
{
	if (value > MAX_value)
	{
		return MAX_value;
	}
	else if (value < -MAX_value)
	{
		return -MAX_value;
	}
	else
	{
		return value;
	}
}

void loop ()
{
	PrintData();
	dt = micros() - lastT;
	lastT = micros();

/*  	if (encoderValR > 2000000000){
		encoderValR -= 2000000000;
		lastEncoderValR -= 2000000000;
	}else if (encoderValR < -2000000000){
		encoderValR += 2000000000;
		lastEncoderValR += 2000000000;
	}
	if (encoderValL > 2000000000){
		encoderValL -= 2000000000;
		lastEncoderValL -= 2000000000;
	}else if (encoderValL < -2000000000){
		encoderValL += 2000000000;
		lastEncoderValL += 2000000000;
	}  */
	MotorControl();
	MotorSync();
}

void MotorControl()
{
	static unsigned long time = 0;
	time += dt;
	TurnLeft(0,0);
	if (time > 2000000)
		TurnLeft(500,0);
	if (time > 5000000)
		TurnLeft(1500,0);
	if (time > 8000000)
		TurnLeft(500,0);
	if (time > 11000000)
		TurnLeft(2000,0);
	
	if (time > 16000000)
	{
		time = 0;
	}
}