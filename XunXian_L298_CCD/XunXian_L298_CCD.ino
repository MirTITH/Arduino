#include <MsTimer2.h>

#define PIN_PWMR 9
#define PIN_AIN2R 11
#define PIN_AIN1R 8
#define ENCODER_AR 3//引发中断
#define ENCODER_BR 5

#define PIN_PWML 10
#define PIN_AIN2L 13
#define PIN_AIN1L 12
#define ENCODER_AL 2//引发中断
#define ENCODER_BL 4
//XY
//红外
#define IR1 0
#define IR2 6
#define IR3 7
#define IR4 A0
#define IR5 A1
#define IR6 A2
#define IR7 A5
#define IR8 A4
#define IR9 A3

#define MAX_TotalSpeed 100000
#define MAX_Motor_Vol 255
#define MAX_deltaVoltL 30//最大左轮修正电压
#define MAX_deltaVoltR 30//最大右轮修正电压


#define GET_SPEED_PERIOD_MS 20

#define RightRatio 1//抵消轮子半径的误差
#define SpeedP (0.05 / 1000000) //每次调整速度的乘数

enum MOTOR_SYNC_MODE {SyncOff, SyncOn} motorSyncMode;//左右电动机同步模式

volatile long encoderValR = 0;
volatile long encoderValL = 0;

volatile long lastEncoderValR = 0;
volatile long lastEncoderValL = 0;

unsigned long lastT = 0;//上次loop微秒值
unsigned long dt = 0;//时间微分，单位微秒

double voltL = 0;
double voltR = 0;
double preVoltL = 0;
double preVoltR = 0;
double deltaVoltPL = 0;//正转时左轮修正电压
double deltaVoltPR = 0;//正转时右轮修正电压
double deltaVoltNL = 0;//反转时左轮修正电压
double deltaVoltNR = 0;//反转时右轮修正电压

int speedR = 0;//实际右轮速度
int speedL = 0;//实际左轮速度

double expSpeedR = 0;//期望右轮速度
double expSpeedL = 0;//期望左轮速度

//double alpha = (double)180 / 180 * PI;

int LastIR[9] = {0};
int IRPort[9] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8, IR9};

void setup()
{
	pinMode(PIN_PWMR, OUTPUT);
	pinMode(PIN_AIN2R, OUTPUT);
	pinMode(PIN_AIN1R, OUTPUT);
	pinMode(PIN_PWML, OUTPUT);
	pinMode(PIN_AIN2L, OUTPUT);
	pinMode(PIN_AIN1L, OUTPUT);

	pinMode(ENCODER_AR, INPUT);
	pinMode(ENCODER_BR, INPUT);
	pinMode(ENCODER_AL, INPUT);
	pinMode(ENCODER_BL, INPUT);
	pinMode(IR1, INPUT);
	pinMode(IR2, INPUT);
	pinMode(IR3, INPUT);
	pinMode(IR4, INPUT);
	pinMode(IR5, INPUT);
	pinMode(IR6, INPUT);
	pinMode(IR7, INPUT);
	pinMode(IR8, INPUT);
	pinMode(IR9, INPUT);

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

	//Serial.begin(115200);
}

/* double LimitSpeed(double speed)
{
	if (speed > MAX_TotalSpeed)
	{
		return MAX_TotalSpeed;
	}
	else if (speed < -MAX_TotalSpeed)
	{
		return -MAX_TotalSpeed;
	}
	else
	{
		return speed;
	}
} */

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

//建议 turnRatio 范围[-1, 1]，执行此函数会自动切换为SyncOn模式
void TurnLeft(double speed, double turnRatio)
{
	motorSyncMode = SyncOn;
	expSpeedR = Limit(RightRatio * speed * (1 - turnRatio), MAX_TotalSpeed);
	expSpeedL = Limit(speed * (1 + turnRatio), MAX_TotalSpeed);

	if (expSpeedL > 0)
	{
		preVoltL = 0.0175 * expSpeedL + 127.54;
	}
	else
	{
		preVoltL = 0.0165 * expSpeedL - 129.96;
	}
	
	if (expSpeedR > 0)
	{
		preVoltR = 0.0177 * expSpeedR + 131.12;
	}
	else
	{
		preVoltR = 0.0184 * expSpeedR - 132.95;
	}
}

void MotorSync()
{
	switch (motorSyncMode)
	{
	case SyncOn:
		//左轮
		if (expSpeedL > 0)
		{
			deltaVoltPL += (expSpeedL - speedL) * SpeedP * dt;
			voltL = preVoltL + deltaVoltPL;

			//限制deltaVolt
			if (deltaVoltPL > MAX_deltaVoltL){
			deltaVoltPL = MAX_deltaVoltL;
			}else if (deltaVoltPL < -MAX_deltaVoltL){
			deltaVoltPL = -MAX_deltaVoltL;}
		}
		else
		{
			deltaVoltNL += (expSpeedL - speedL) * SpeedP  * dt;
			voltL = preVoltL + deltaVoltNL;

			//限制deltaVolt
			if (deltaVoltNL > MAX_deltaVoltL){
			deltaVoltNL = MAX_deltaVoltL;
			}else if (deltaVoltNL < -MAX_deltaVoltL){
			deltaVoltNL = -MAX_deltaVoltL;}
		}
		
		//右轮
		if (expSpeedR > 0)
		{
			deltaVoltPR += (expSpeedR - speedR) * SpeedP  * dt;
			voltR = preVoltR + deltaVoltPR;

			//限制deltaVolt
			if (deltaVoltPR > MAX_deltaVoltR){
			deltaVoltPR = MAX_deltaVoltR;
			}else if (deltaVoltPR < -MAX_deltaVoltR){
			deltaVoltPR = -MAX_deltaVoltR;}
		}
		else
		{
			deltaVoltNR += (expSpeedR - speedR) * SpeedP  * dt;
			voltR = preVoltR + deltaVoltNR;

			//限制deltaVolt
			if (deltaVoltNR > MAX_deltaVoltR){
			deltaVoltNR = MAX_deltaVoltR;
			}else if (deltaVoltNR < -MAX_deltaVoltR){
			deltaVoltNR = -MAX_deltaVoltR;}
		}

		//限制volt
		if (voltL > MAX_Motor_Vol){
			voltL = MAX_Motor_Vol;
		}else if (voltL < -MAX_Motor_Vol){
			voltL = -MAX_Motor_Vol;}

		if (voltR > MAX_Motor_Vol){
			voltR = MAX_Motor_Vol;
		}else if (voltR < -MAX_Motor_Vol){
			voltR = -MAX_Motor_Vol;}

		analogWrite(PIN_PWMR, fabs(voltR));
		analogWrite(PIN_PWML, fabs(voltL));

		if (voltL > 0)
		{
			digitalWrite(PIN_AIN2L, LOW);
			digitalWrite(PIN_AIN1L, HIGH);
		}
		else if (voltL < 0)
		{
			digitalWrite(PIN_AIN2L, HIGH);
			digitalWrite(PIN_AIN1L, LOW);
		}
		else
		{
			digitalWrite(PIN_AIN2R, LOW);
			digitalWrite(PIN_AIN1R, LOW);
		}

		if (voltR > 0)
		{
			digitalWrite(PIN_AIN2R, LOW);
			digitalWrite(PIN_AIN1R, HIGH);
		}
		else if (voltR < 0)
		{
			digitalWrite(PIN_AIN2R, HIGH);
			digitalWrite(PIN_AIN1R, LOW);
		}
		else
		{
			digitalWrite(PIN_AIN2R, LOW);
			digitalWrite(PIN_AIN1R, LOW);
		}
		break;
	case SyncOff:
		analogWrite(PIN_PWMR, fabs(voltR));
		analogWrite(PIN_PWML, fabs(voltL));

		if (voltL > 0)
		{
			digitalWrite(PIN_AIN2L, LOW);
			digitalWrite(PIN_AIN1L, HIGH);
		}
		else if (voltL < 0)
		{
			digitalWrite(PIN_AIN2L, HIGH);
			digitalWrite(PIN_AIN1L, LOW);
		}
		else
		{
			digitalWrite(PIN_AIN2R, LOW);
			digitalWrite(PIN_AIN1R, LOW);
		}

		if (voltR > 0)
		{
			digitalWrite(PIN_AIN2R, LOW);
			digitalWrite(PIN_AIN1R, HIGH);
		}
		else if (voltR < 0)
		{
			digitalWrite(PIN_AIN2R, HIGH);
			digitalWrite(PIN_AIN1R, LOW);
		}
		else
		{
			digitalWrite(PIN_AIN2R, LOW);
			digitalWrite(PIN_AIN1R, LOW);
		}
		break;
	default:
		break;
	}
}

void PrintData()
{
	Serial.print("\nSL ");
	Serial.print(speedL);
	Serial.print(",SR ");
	Serial.print(speedR);
	Serial.print(",ExpL ");
	Serial.print(expSpeedL);
	Serial.print(",ExpR ");
	Serial.print(expSpeedR);
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
	
	// Serial.print(" dt= ");
	// Serial.print(dt / 100);
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

//将IR1~9存入数组IRGroup
void SaveIR(int * IRGroup)
{
	IRGroup[0] = digitalRead(IR1);
	IRGroup[1] = digitalRead(IR2);
	IRGroup[2] = digitalRead(IR3);
	IRGroup[3] = digitalRead(IR4);
	IRGroup[4] = digitalRead(IR5);
	IRGroup[5] = digitalRead(IR6);
	IRGroup[6] = digitalRead(IR7);
	IRGroup[7] = digitalRead(IR8);
	IRGroup[8] = digitalRead(IR9);
}

//返回平均IR值，Type值为0时从传感器读入，否则从传入的数组中读入
double IRAvgVal(int* Type)
{
	int total = 0;
	int weightedNum = 0;
	if (Type == 0)
	{
		total = IRSignNum(0);

		if (total == 0)
		{
			return 5;
		}
		else
		{
			for (int i = 0; i < 9; i++)
			{
				weightedNum += (i + 1) * digitalRead(IRPort[i]);
			}
			return (double)weightedNum / total;
		}
	}
	else
	{
		total = IRSignNum(Type);
		if (total == 0)
		{
			return 5;
		}
		else
		{
			for (int i = 0; i < 9; i++)
			{
				weightedNum += (i + 1) * Type[i];
			}
			return (double)weightedNum / total;
		}
	}
}

//返回有信号的IR传感器数，传入0时从传感器读取，否则从数组中读取
int IRSignNum(int* Type)
{
	int Result = 0;
	if (Type == 0)
	{
		for (int i = 0; i < 9; i++)
		{
			Result += digitalRead(IRPort[i]);
		}
	}
	else
	{
		for (int i = 0; i < 9; i++)
		{
			Result += Type[i];
		}
	}
	return Result;
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

void SpinLeft()
{
	
}

void loop()
{
	//PrintData();
	dt = micros() - lastT;
	lastT = micros();

 	if (encoderValR > 2000000000){
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
	} 
	MotorControl();
	MotorSync();
}

void MotorControl()
{
	static double MAX_speedControl = 1500;
	static double turnP = 1;//转弯比例系数
	static double turnD = 0.05;//转弯微分系数
	static double atanValue = 2;
	static unsigned long DDt = 150000;//微分▲t，单位微秒
	static double MAX_turnRatio = 1.01;//最大转弯系数
	static double MAX_turnRatioD = 0.5;//最大微分项的值

	static double turnRatioP = 0;//比例项
	static double turnRatioD = 0;//微分项
	static double turnRatio = 0;//总转弯系数
	static double lastDAvg = 5;//上一个▲t的平均值
	static double DAvg = 0;//当前▲t的平均值
	static double speedControl = MAX_speedControl;//控制的速度
	static unsigned long sumDt = 0;
	
	if (IRSignNum(0) != 0)
	{
		//SaveIR(LastIR);
		//比例项计算
		turnRatioP = (double)turnP * atan(atanValue * (IRAvgVal(0) - 5) / 4) / atan(atanValue);

		//微分项计算
		sumDt += dt;
		if (sumDt <= DDt)
		{
			DAvg += IRAvgVal(0) * dt / DDt;
		}
		else
		{
			turnRatioD = (DAvg - lastDAvg) * 1000000 / DDt;
			lastDAvg = DAvg;
			sumDt = 0;
			DAvg = 0;
		}
		turnRatioD = Limit(turnRatioD, MAX_turnRatioD);
		turnRatio = turnRatioD + turnRatioP;
		turnRatio = Limit(turnRatio, MAX_turnRatio);

		if (IRSignNum(0) > 3)
		{
			if (turnRatioP < 0)
			{
				turnRatio = -1.1;
			}
			else
			{
				turnRatio = 1.1;
			}
			speedControl = 1000;
		}
	}
	
	TurnLeft((double)speedControl * (1 - 0.4 * fabs(turnRatio)), turnRatio);

	if (speedControl < MAX_speedControl)
	{
		speedControl += (double)dt / 3000;
	}

/* 	static long counter = 0;
	static int speed = 0;
	if (fabs(expSpeedR - speedR) <= 50)
	{
		counter += dt;
		if (counter > 1500000)
		{
			speed -= 25;
			counter = 0;
			PrintData();
		}
	}
	else
	{
		counter -= dt;
	}
	TurnLeft(speed, -1); */
/* 	static unsigned long lT1 = 0;
	if (micros() - lT1 > 20000000)
	{
		lT1 = micros();
	}
	else if (micros() - lT1 > 16000000)
	{
		TurnLeft(-1000, 0);
	}
	else if (micros() - lT1 > 12000000)
	{
		TurnLeft(0, 0);
	}
	else if (micros() - lT1 > 8000000)
	{
		TurnLeft(1000, 0);
	}
	else if (micros() - lT1 > 4000000)
	{
		TurnLeft(500, 0);
	}
	 */
}