#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    mobile1,        sensorPotentiometer)
#pragma config(Sensor, in2,    mobile2,        sensorPotentiometer)
#pragma config(Sensor, in3,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  elevator,       sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  solenoid,       sensorDigitalOut)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           bar1,          tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           rf,            tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,           rb,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           lf,            tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port5,           lb,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           goal1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           goal2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           lift1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          bar2,          tmotorVex393_HBridge, openLoop)
#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)
#include "Vex_Competition_Includes.c"

int reverse = 1;
int half = 1;
int rl=0;
int hl=0;
int sl=0;
int ll=0;
bool lower = false;
long error[2] = {0, 0}
long i[2] = {0,0};

void RecordingdriveCode()
{
	if(!lower){
			motor[goal1] = (127 * vexRT[Btn6U]) + (-127 * vexRT[Btn6D]);
			motor[goal2] = (127 * vexRT[Btn6U]) + (-127 * vexRT[Btn6D]);
  }

    else{
        error[0] = 730 - SensorValue(mobile1);
        error[1] = 345 - SensorValue(mobile2);
        i[0] = abs(i[0] + error[0]) < 0 ? i[0] + error[0] : sgn(i[0] + error[0])*0;
        i[1] = abs(i[1] + error[1]) < 0 ? i[0] + error[1] : sgn(i[0] + error[1])*0;
        motor[port7] = error[0]*0.3 + i[0]*0.04;
        motor[port6] = error[1]*0.3 + i[1]*0.04;
    }
    motor[lf] = reverse * (vexRT[Ch3] + reverse*(vexRT[Ch1]+ vexRT[Ch4]))/half;
		motor[lb] = reverse * (vexRT[Ch3] + reverse*(vexRT[Ch1]+ vexRT[Ch4]))/half;
		motor[rf] = reverse * (vexRT[Ch3] - reverse*(vexRT[Ch1]+ vexRT[Ch4]))/half;
		motor[rb] = reverse * (vexRT[Ch3] - reverse*(vexRT[Ch1]+ vexRT[Ch4]))/half;



		motor[lift1] = (127 * vexRT[Btn5U]) + (-127 * vexRT[Btn5D]);
		motor[lift2] = (127 * vexRT[Btn5U]) + (-127 * vexRT[Btn5D]);

		motor[bar1] = vexRT[Ch2];
		motor[bar2] = vexRT[Ch2];

		if(vexRT[Btn8D]!=rl && rl){
			reverse *= -1;
		}
		rl = vexRT[Btn8D];
		if(vexRT[Btn7D]!=hl && hl){
			half == 1 ? half = 2 : half = 1;
		}
		hl = vexRT[Btn7D];
		if (vexRT[Btn8R]!=sl && sl) {
				SensorValue[solenoid] = !SensorValue[solenoid];
		}
		sl = vexRT[Btn8R];

		if(vexRT[Btn8L]!=ll && ll){
					lower = !lower;
				}
		ll = vexRT[Btn8L];
}
task autonomous()
{
	//copy and paste here
}

int loopTime;
int waitTime=0;
int oldMotorValue[10];
const int LOOP_TIME_PER_MOTOR_CHANGE=20;
const float MAX_MOTOR_CHANGE_PER_MS=1.0;
void slewRate()
{
	for(int j=0;j<10;j++)
	{
		if(abs(motor[j]-oldMotorValue[j])>MAX_MOTOR_CHANGE_PER_MS*loopTime)
		{
			if(motor[j]>oldMotorValue[j])
				motor[j]+=MAX_MOTOR_CHANGE_PER_MS*loopTime;
			else
				motor[j]-=MAX_MOTOR_CHANGE_PER_MS*loopTime;
		}
	}
}
task record()
{
	clearDebugStream();
	waitTime=0;
	writeDebugStream("recordingBattery=%d",nAvgBatteryLevel);
	writeDebugStreamLine(";");
	while(true)
	{
		RecordingdriveCode();
		loopTime=0;
		for(int j=0;j<10;j++)
		{
			loopTime+=oldMotorValue[j]!=motor[j]*LOOP_TIME_PER_MOTOR_CHANGE;
		}
		if(loopTime<LOOP_TIME_PER_MOTOR_CHANGE)
			loopTime=LOOP_TIME_PER_MOTOR_CHANGE;
		slewRate();
		for(int j=0;j<10;j++)
		{
			if(oldMotorValue[j]!=motor[j])
			{
				oldMotorValue[j]=motor[j];
				if(waitTime>0)
				{
					writeDebugStream("wait1Msec(wait(%d",waitTime);
					writeDebugStreamLine("));");
				}
				writeDebugStream("motor[%d",j);
				writeDebugStream("]=%d",oldMotorValue[j]);
				writeDebugStreamLine(";");
				waitTime=0;
			}
		}
		waitTime+=loopTime+1;
		wait1Msec(loopTime);
	}
}
void pre_auton()
{
  bStopTasksBetweenModes = true;
}
task usercontrol()
{
	startTask(record);
	while (true)
	wait1Msec(1);
}
