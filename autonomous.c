#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, in2,    manipulator,    sensorPotentiometer)
#pragma config(Sensor, in3,    base,           sensorPotentiometer)
#pragma config(Sensor, in4,    lift,           sensorPotentiometer)
#pragma config(Sensor, dgtl1,  L,              sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  sonar,          sensorSONAR_inch)
#pragma config(Sensor, dgtl11, R,              sensorQuadEncoder)
#pragma config(Motor,  port1,           rollers,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           liftL,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           manipulatorL,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           baseLfront,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           baseLback,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           baseRfront,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           baseRback,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           manipulatorR,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           liftR,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          mogo,          tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

void driveL(int speed){
	motor[baseLfront] = speed;
  motor[baseLback] = speed;
}

void driveR(int speed){
	motor[baseRfront] = speed;
  motor[baseRback] = speed;
}

void dr4b(int speed){
	motor[liftL] = speed;
	motor[liftR] = speed;
}
void translational(int speed){
	motor[manipulatorL] = speed;
	motor[manipulatorR] = speed;
}

void drive(int speed){
	motor[baseRfront] = speed;
  motor[baseRback] = speed;
  motor[baseLfront] = speed;
  motor[baseLback] = speed;
}

void pDrive(int target){
  SensorValue[L] = 0;
  SensorValue[R] = 0;

  double kp = 0.15;
  int kc = 60;
  int brake = -60;
  int dir = 1; //forward
  if(target < 0){ //backward
    dir = 0;
    brake = -brake;
  }
	int left, right, errorL, errorR, speedL, speedR;
  while(1){
    left = SensorValue(L);
    right = SensorValue(R);
    errorL = target-left;
    errorR = target - right;
    speedL = errorL * kp;
    speedR = errorR *kp;

    //speed normalization
    if(dir == 1 && speedL < kc) speedL = kc;
    if(dir == 0 && speedL > -kc) speedL = -kc;
    if(dir == 1 && speedR < kc) speedR = kc;
    if(dir == 0 && speedR > -kc) speedR = -kc;

    driveL(speedL);
    driveR(speedR)

    if(dir == 0 && errorL > 0) break;
    if(dir == 1 && errorL < 0) break;
  }
  drive(-brake);
  delay(200);
  drive(0); // stop drive
}

bool autoRight = false;
void pTurn(int sp){
  if(autoRight == true) sp = -sp; // inverted turn speed for right auton
  double kp = 0.08;
/*
  if(abs(sp - SensorValue[gyro]) > 110) kp = 1.8;
  if(abs(sp - SensorValue[gyro]) > 150) kp = 1.2;
  if(abs(sp - Sensorvalue[gyro]) > 180) kp = .8;
  */
  int kc = 45;
  int brake = -45;

  //set direction
  int dir = 0; //left
  if(sp - SensorValue[gyro] < 0) dir = 1;
  if(dir == 1) brake = -brake;


  while(1){
    int sv = SensorValue[gyro]; // get sensor

    //calculate speed
    int error = sp-sv;
    int speed = error*kp;

    //velocity normalization
    if(speed > 127) speed = 127;
    if(speed < -127) speed = -127;

    //kc enforcement
    if(dir == 0 && speed < kc) speed = kc;
    if(dir == 1 && speed > -kc) speed = -kc;

    //end loop check
    if(dir == 0 && error <= 0) break;
    if(dir == 1 && error >= 0) break;

    driveL(-speed);
    driveR(speed);

    delay(20);
  }
  driveL(brake);
  driveR(-brake);
  delay(100);
  drive(0); // stop drive
}

task mogo_down(){
	while(SensorValue[base] < 3200){
		motor[mogo] = -127;
	}
	motor[mogo] = 0;
}

task mogo_up(){
	while(SensorValue[base] > 1150){
		motor[mogo] = 127;
	}
	motor[mogo] = 15;
}
task lift_raise(){
	while(SensorValue(lift) > 1500){
		dr4b(75);
	}
	dr4b(-10);
	wait1Msec(200);
	dr4b(0);
}
task preload(){
	motor[rollers] = -127;
	wait1Msec(100);
	motor[rollers] = -30;
}

task manipulator_in(){
	while(SensorValue(manipulator) > 100){
					translational(-127);
				}
	translational(-10);
}

task manipulator_out(){
	while(SensorValue(manipulator) < 2100){
				translational(127);
			}
	translational(10);
}
task drop_preload(){
	while(SensorValue[lift] < 1870){
	   dr4b(-127);
	   motor[rollers] = 127;
	 }
	 dr4b(0);
	 wait1Msec(250);

	 while(SensorValue(lift) > 1700){
		dr4b(75);
		}
		dr4b(-10);
		wait1Msec(200);
		dr4b(0);
}
void lift_lower(int target){
	while(SensorValue[lift] < target){
	   dr4b(-127);
	 }
	 dr4b(0);
}
task third(){
	while(SensorValue(lift) > 1500){
		dr4b(75);
	}
	dr4b(-10);
	wait1Msec(200);
	dr4b(0);
	 startTask(manipulator_in);
	 wait1Msec(250);
	 lift_lower(1630);
	 motor[rollers] = 127;
	 wait1Msec(300);
	 startTask(lift_raise);
}
task main()
{

	autoRight = True;
	SensorType[in1] = sensorNone;
 	wait1Msec(1000);
	 SensorType[in1] = sensorGyro;
	 wait1Msec(2000);
	 startTask(preload);
	 startTask(lift_raise);
	 startTask(manipulator_in);
	 startTask(mogo_down);
	 pdrive(740);
	 startTask(mogo_up);
	 wait1Msec(1250);
	 startTask(drop_preload);
	 pdrive(80);
	 wait1Msec(500);
	 startTask(manipulator_out);
	 motor[rollers] = -127;
	 lift_lower(1950);
	 wait1Msec(400);
	 motor[rollers] = -30;
	 startTask(lift_raise);
	 wait1Msec(450);
	 startTask(manipulator_in);
	 wait1Msec(100);
	 lift_lower(1800);
	 motor[rollers] = 127;
	 startTask(lift_raise);
	 pdrive(60);
	 startTask(manipulator_out);
	 motor[rollers] = -127;
	 lift_lower(1950);
	 wait1Msec(500);
	 motor[rollers] = -30;
	 startTask(third);
	 pdrive(-(1100));
	 pturn(450);
	 pdrive(-250);
	 pturn(1350);
	 pdrive(250);
	 while(SensorValue[base] < 3200){
		motor[mogo] = -127;
	}
	motor[mogo] = 0;
	drive(-50);
	wait1Msec(250);
	startTask(mogo_up);
	 drive(-127);
	 wait1Msec(750);
	 drive(0);

}
