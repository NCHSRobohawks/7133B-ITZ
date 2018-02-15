#pragma config(Sensor, in1,    liftL,          sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, in3,    liftR,          sensorPotentiometer)
#pragma config(Sensor, in4,    manipulator,    sensorPotentiometer)
#pragma config(Sensor, dgtl1,  L,              sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  R,              sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  ultrasonic,     sensorSONAR_inch)
#pragma config(Motor,  port1,           rollers,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           liftL1,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           liftL2,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           driveR,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port5,           manipulatorL,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           manipulatorR,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           liftR1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           liftR2,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           driveL,        tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port10,          mogo,          tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

float ticksPerFoot = 688;

/*
Error Mapping:
0 - Left drive
1 - Right drive
2 - Yaw rate Gyro
3 - Left potentiometer
4 - Right potentiometer
5 - Manipulator potentiometer
*/
float error[6] =  {0,0,0,0,0,0}
float pError[6] =  {0,0,0,0,0,0};
float I[6] = {0,0,0,0,0,0};
/*
Constant Mapping:
0 - Drive
1 - Gyro/Turning
2 - Lift
3 - Manipulator
*/
float kP[4] = {0,0,0,0};
float kI[4] = {0,0,0,0};
float kD[4] = {0,0,0,0};


void straighten(int target){
	error[2] = target - SensorValue(gyro);
	I[2] += error[2];
	if(error[2] > 50 || abs(error[2]) > 10){
		I[2]  = 0;
	}
	motor[driveL] += -(error[2] * kP[1] + I[2] * kI[1] + (error[2] - pError[2]) * kD[1]);
	motor[driveR] += error[2] * kP[1] * kP[1] + I[2] * kI[1] + + (error[2] - pError[2]) * kD[1];
	pError[2] = error[2];
}

void pd_base(int target, int degrees10){
	error[0] = target - SensorValue(L);
	error[1] = target - SensorValue(R);
	I[0] += error[0];
	if(abs(error[0]) > 50 || abs(error[0]) < 5){
		I[0] = 0;
	}
	I[1] += error[0];
	if(abs(error[1]) > 50 || abs(error[1]) < 5){
		I[1] = 0;
	}
	motor[driveL] = error[0] * kP[0] + (error[0] - pError[0]) * kD[0];
	motor[driveR] = error[1]* kP[0] + (error[1] - pError[1]) * kD[0];
	pError[0] = error[0];
	pError[1] = error[1];
	straighten(degrees10);
}

void pid_turn(int target){
	error[2] = target - SensorValue(gyro);
	I[2] += error[2];
	if(error[2] > 35 || abs(error[2]) == 0){
		I[2]  = 0;
	}
	motor[driveL] = -(error[2] * kP[1] + I[2] * kI[1] + (error[2] - pError[2]) * kD[1]);
	motor[driveR] = error[2] * kP[1] * kP[1] + I[2] * kI[1] + + (error[2] - pError[2]) * kD[1];
	pError[2] = error[2];
}



void pid_lift(int targetL, int targetR){
	error[3] = SensorValue[liftL]- targetL;
	error[4] = SensorValue[liftR]- targetR;
	I[3] += error[3];
	if(abs(error[3]) > 200 || abs(error[3]) < 10){
		I[3]  = 0;
	}
	I[4] += error[4];
	if(abs(error[4]) > 200 || abs(error[4]) < 10){
		I[4]  = 0;
	}
	motor[liftL1] = error[3] * kP[2] + I[3] * kI[2] + (error[3] - pError[3])*kD[2];
	motor[liftR1] = error[3] * kP[2] + I[3] * kI[2] + (error[3] - pError[3])*kD[2];
	motor[liftR2] = error[4] * kP[2] + I[3] * kI[2] + (error[4] - pError[4])*kD[2];
	motor[liftR2] = error[4] * kP[2] + I[3] * kI[2] + (error[4] - pError[4])*kD[2];
	pError[3] = error[3];
	pError[4] = error[4];
}

void pid_manipulator(int target){
	error[5] = target - SensorValue(manipulator);
	I[5] += error[5]
	if(abs(error[5]) < 20){
		I[5]  = 0;
}
	motor[manipulatorL] = error[5] *kP[3] + I[4] * kI[3] + (error[5] - pError[5]) * kD[3];
	motor[manipulatorR] = error[5] *kP[3] + I[4] * kI[3] + (error[5] - pError[5]) * kD[3];
	pError[5] = error[5];
}

void move(int target){
	SensorValue[L] = 0;
	SensorValue[R] = 0;
	pError[0] = 0;
	pError[1] = 0;
	int degrees = SensorValue[in5]
	while(abs(SensorValue(L) - target) > 30 && abs(SensorValue(R) - target) > 30){
		pd_base(target, degrees);
	}
	motor[driveL] = 0;
	motor[driveR] = 0;
}

void turn(int degrees10){
	if(SensorType[in5] != sensorGyro){
		SensorType[in5] = sensorNone;
		wait1Msec(1000);
		SensorType[in5] = sensorGyro;
		wait1Msec(2000);
		}
	while(abs(SensorValue(in5)) < degrees10){

		pid_turn(degrees10);
	}
}

void lift_set(int targetL, int targetR){
	I[4] = 0;
	while((Sensorvalue[liftR] - targetR) > 15){
		pid_lift(targetL, targetR)
		wait1Msec(25);
	}
}

void manipulator_set(int target){
	I[2] = 0;
	while((Sensorvalue[manipulator] - target) > 15){
		pid_manipulator(target)
		wait1Msec(25);
	}
}
void pre_auton()
{
  bStopTasksBetweenModes = true;
}

task autonomous()
{
  AutonomousCodePlaceholderForTesting();
}

const unsigned short trueSpeed[128] =
{
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 21 , 21, 21, 22, 22, 22, 23, 24, 24,
	25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
	28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
	33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
	37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
	41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
	46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
	52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
	71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
	80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
	88, 89, 89, 90, 90,127,127,127
};
int speedLeft;
int speedRight;
int speedManipulator;

void driveSpeed(){
    if(speedLeft > 127) speedLeft = 127;
    if(speedLeft < -127) speedLeft = -127;
		if(speedRight > 127) speedRight = 127;
    if(speedRight < -127) speedRight = -127;
    int absSpeedL = trueSpeed[abs(speedLeft)]*(speedLeft/abs(speedLeft+0.0001));
		int absSpeedR = trueSpeed[abs(speedRight)]*(speedRight/abs(speedRight+0.0001));
    motor[driveL] = absSpeedL;
    motor[driveR] = absSpeedR;
}

void manipulator_speed(){
    if(speedManipulator > 127) speedManipulator = 127;
    if(speedManipulator < -127) speedManipulator = -127;
		int absSpeed = trueSpeed[abs(speedManipulator)]*(speedManipulator/abs(speedManipulator+0.0001));
    motor[manipulatorL] = absSpeed;
    motor[manipulatorR] = absSpeed;
}

int manipulator_target;
//Equation that constantly converts current lift value to potentiometer target for four bar
task lift_to_manipulator(){
	while(true){
		manipulator_target = SensorValue(L);
		wait1Msec(20);
	}
}

//Task that handles all the advanced and basic lift/four bar control
task lift_control(){
	startTask(lift_to_manipulator);
	while(true){
		motor[liftL1] = (vexRT[Btn6U] * 127) + (vexRT[Btn6D] * -127);
		motor[liftL2] = (vexRT[Btn6U] * 127) + (vexRT[Btn6D] * -127);
		motor[liftR1] = (vexRT[Btn6U] * 127) + (vexRT[Btn6D] * -127);
		motor[liftR2] = (vexRT[Btn6U] * 127) + (vexRT[Btn6D] * -127);
		if(vexRT[Ch2] > 10){
			speedManipulator= vexRT[Ch2];
			manipulator_speed();
		}
		else if(vexRT[Btn8D]){
			pid_manipulator(manipulator_target);
		}
		motor[rollers] = (vexRT[Btn7U] * 127) + (vexRT[Btn7D] * -157) + 30;
	}
	}


task usercontrol()
{
	startTask(lift_control);
  while (true)
  {
		motor[mogo] = (vexRT[Btn5UXmtr2] * 147) + (vexRT[Btn5DXmtr2] * -127) - 20;
		speedLeft = vexRT[Ch3Xmtr2] + vexRT[Ch4Xmtr2];
		speedRight = vexRT[Ch3Xmtr2] -  vexRT[Ch4Xmtr2];
		driveSpeed();
  }
}
