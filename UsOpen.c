#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    gyro,           sensorNone)
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

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

// Include the lcd button get utility function
#ifndef  _GETLCDBUTTONS
#define  _GETLCDBUTTONS

// Some utility strings
#define LEFT_ARROW  247
#define RIGHT_ARROW 246
static  char l_arr_str[4] = { LEFT_ARROW,  LEFT_ARROW,  LEFT_ARROW,  0};
static  char r_arr_str[4] = { RIGHT_ARROW, RIGHT_ARROW, RIGHT_ARROW, 0};

/*-----------------------------------------------------------------------------*/
/*  This function is used to get the LCD hutton status but also acts as a      */
/*  "wait for button release" feature.                                         */
/*  Use it in place of nLcdButtons.                                            */
/*  The function blocks until a button is pressed.                             */
/*-----------------------------------------------------------------------------*/

// Little macro to keep code cleaner, masks both disable/ebable and auton/driver
#define vexCompetitionState (nVexRCReceiveState & (vrDisabled | vrAutonomousMode))

TControllerButtons
getLcdButtons()
{
    TVexReceiverState   competitionState = vexCompetitionState;
    TControllerButtons  buttons;

    // This function will block until either
    // 1. A button is pressd on the LCD
    //    If a button is pressed when the function starts then that button
    //    must be released before a new button is detected.
    // 2. Robot competition state changes

    // Wait for all buttons to be released
    while( nLCDButtons != kButtonNone ) {
        // check competition state, bail if it changes
        if( vexCompetitionState != competitionState )
            return( kButtonNone );
        wait1Msec(10);
        }

    // block until an LCD button is pressed
    do  {
        // we use a copy of the lcd buttons to avoid their state changing
        // between the test and returning the status
        buttons = nLCDButtons;

        // check competition state, bail if it changes
        if( vexCompetitionState != competitionState )
            return( kButtonNone );

        wait1Msec(10);
        } while( buttons == kButtonNone );

    return( buttons );
}
#endif

//Global hold the auton selection
static int MyAutonomous = 0;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

#define MAX_CHOICE  9

void
LcdAutonomousSet( int value, bool select = false )
{
    // Cleat the lcd
    clearLCDLine(0);
    clearLCDLine(1);

    // Display the selection arrows
    displayLCDString(1,  0, l_arr_str);
    displayLCDString(1, 13, r_arr_str);

    // Save autonomous mode for later if selected
    if(select)
        MyAutonomous = value;

    // If this choice is selected then display ACTIVE
    if( MyAutonomous == value )
        displayLCDString(1, 5, "ACTIVE");
    else
        displayLCDString(1, 5, "select");

    // Show the autonomous names
    switch(value) {
        case    0:
            displayLCDString(0, 0, "20 pt 3 cones Right");
            break;
        case    1:
            displayLCDString(0, 0, "20 pt 3 cones Left");
            break;
        case    2:
            displayLCDString(0, 0, "20 pt 2 cones Right");
            break;
        case    3:
            displayLCDString(0, 0, "20 pt 2 cones Left");
            break;
        case    4:
            displayLCDString(0, 0, "5 pt 3 cones Right");
            break;
        case    5:
            displayLCDString(0, 0, "5 pt 3 cones Left");
            break;
        case    6:
            displayLCDString(0, 0, "5 pt 2 cones Right");
            break;
        case    7:
            displayLCDString(0, 0, "5 pt.= 2 cones Left");
            break;
        case    8:
            displayLCDString(0, 0, "The Final Solution");
            break;
        default:
            displayLCDString(0, 0, "Unknown");
            break;
        }
}

/*-----------------------------------------------------------------------------*/
/*  Rotate through a number of choices and use center button to select         */
/*-----------------------------------------------------------------------------*/

void
LcdAutonomousSelection()
{
    TControllerButtons  button;
    int  choice = 0;

    // Turn on backlight
    bLCDBacklight = true;

    // diaplay default choice
    LcdAutonomousSet(0);

    while(bIfiRobotDisabled)
        {
        // this function blocks until button is pressed
        button = getLcdButtons();

        // Display and select the autonomous routine
        if( ( button == kButtonLeft ) || ( button == kButtonRight ) ) {
            // previous choice
            if( button == kButtonLeft )
                if( --choice < 0 ) choice = MAX_CHOICE;
            // next choice
            if( button == kButtonRight )
                if( ++choice > MAX_CHOICE ) choice = 0;
            LcdAutonomousSet(choice);
            }

        // Select this choice
        if( button == kButtonCenter )
            LcdAutonomousSet(choice, true );

        // Don't hog the cpu !
        wait1Msec(10);
        }
}



void pre_auton()
{
	bStopTasksBetweenModes = true;
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "Calibrating Gyro");
  SensorType[in1] = sensorNone;
 	wait1Msec(1000);
	SensorType[in1] = sensorGyro;
	wait1Msec(2000);
	LcdAutonomousSelection();


	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

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
  int kc = 60;
  int brake = 60;

  //set direction
  int dir = 0; //left
  if(sp - SensorValue[gyro] < 0) dir = 1;
  if(dir == 1) brake = -brake;


  while(1){
    int sv = SensorValue[gyro];

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
	motor[mogo] = -10;
}

task mogo_up(){
	while(SensorValue[base] > 1150){
		motor[mogo] = 127;
	}
	motor[mogo] = 15;
}
task lift_raise(){
	while(SensorValue(lift) > 2800){
		dr4b(127);
	}
	dr4b(10);
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
bool finished;
task drop_preload(){
	finished = false
	while(SensorValue[lift] < 3100){
	   dr4b(-127);
	 }
	 motor[rollers] = 127;
	 dr4b(0);
	 wait1Msec(250);
	 while(SensorValue(lift) > 3000){
		dr4b(75);
		}
		dr4b(-10);
		wait1Msec(200);
		dr4b(0);
		finished = true;
}
void lift_lower(int target){
	while(SensorValue[lift] < target){
	   dr4b(-127);
	 }
	 dr4b(0);
}

task third(){
	finished = false;
	while(SensorValue(lift) >2800){
		dr4b(127);
	}
	dr4b(10);
	wait1Msec(200);
	dr4b(0);
	 startTask(manipulator_in);
	 wait1Msec(250);
	 lift_lower(2900);
	 motor[rollers] = 127;
	 wait1Msec(450);
	 while(SensorValue(lift) > 2800){
		dr4b(127);
	}
	dr4b(10);
	finished = true;
}
task second(){
	finished = false;
	 while(SensorValue(lift) > 2800){
		dr4b(127);
	}
	dr4b(10);
	wait1Msec(100);
	dr4b(0);
	 startTask(manipulator_in);
	 wait1Msec(100);
	 lift_lower(3000);
	 motor[rollers] = 127;
	 wait1Msec(300);
	 while(SensorValue(lift) > 2800){
		dr4b(127);
	}
	dr4b(0);
	finished = true;
}

bool three = false;

void nearZone(){
	SensorValue[gyro] = 0;
	int delta = nSysTime;
	 startTask(preload);
	 startTask(lift_raise);
	 startTask(manipulator_in);
	 startTask(mogo_down);
	 pdrive(800);
	 startTask(mogo_up);
	 wait1Msec(1250);
	 startTask(drop_preload);
	 pdrive(50);
	 while(!finished);
	 startTask(manipulator_out);
	 motor[rollers] = -127;
	 lift_lower(3300);
	 wait1Msec(300);
	 motor[rollers] = -30;
	 startTask(second);
	 if(three){
		 pdrive(40);
		 while(!finished);
		 startTask(manipulator_out);
		 motor[rollers] = -127;
		 lift_lower(3300);
		 wait1Msec(250);
		 motor[rollers] = -30;
		 startTask(third);
		 pdrive(-(900));
	}
	else{
		pdrive(-700);
	}
		SensorValue[gyro] = 0;
	 pturn(1800);
	 while(SensorValue[base] < 3200){
		motor[mogo] = -127;
	}
	motor[mogo] = 0;
	drive(-127)
	wait1Msec(250);
	startTask(mogo_up);
	wait1Msec(250);
	drive(0);
	delta = nSysTime-delta;
	clearLCDLine(0);
	clearLCDLine(1);
	writeDebugStreamLine("Time: %d", delta);
	writeDebugStreamLine("Sys Time: %d", nSysTime);
}
void farZone(){
	SensorValue[gyro] = 0;
	 int delta = nSysTime;
	 startTask(preload);
	 startTask(lift_raise);
	 startTask(manipulator_in);
	 startTask(mogo_down);
	 pdrive(800);
	 startTask(mogo_up);
	 wait1Msec(1250);
	 startTask(drop_preload);
	 pdrive(50);
	 while(!finished);
	 startTask(manipulator_out);
	 motor[rollers] = -127;
	 lift_lower(3300);
	 wait1Msec(300);
	 motor[rollers] = -30;
	 startTask(second);

	 if(three){
			pdrive(40);
			while(!finished);
			startTask(manipulator_out);
		 	motor[rollers] = -127;
			lift_lower(3300);
			wait1Msec(250);
			motor[rollers] = -30;
			startTask(third);
			pdrive(-(1040));
		}

		else{pdrive(-1000);}

	 pturn(450);
	 pdrive(-260);
	 SensorValue[gyro] = 0;
	 pturn(900);
	 startTask(mogo_down);
	 pdrive(290);

	motor[mogo] = 0;
	drive(-127)
	wait1Msec(100);
	startTask(mogo_up);
	wait1Msec(500);
	drive(0);
	delta = nSysTime-delta;
	clearLCDLine(0);
	clearLCDLine(1);
	writeDebugStreamLine("Time: %d", delta);
	writeDebugStreamLine("Sys Time: %d", nSysTime);
}

void theFinalSolution(){
	drive(127);
}

task limit(){
	int init = nSysTime;
	int delta = 0;
	while(delta < 15000){delta = delta - init;}
	stopTask(autonomous);
	allMotorsOff();
}
task autonomous()
{
  switch( MyAutonomous ) {
        case    0:
            three = true;
            autoRight = true;
            farZone();
            break;

        case    1:
            three = true;
            autoRight = false;
            farZone();
            break;
        case    2:
            three = false;
            autoRight = true;
            farZone();
            break;
        case    3:
            three = false;
            autoRight = false;
            farZone();
            break;
        case    4:
            three = true;
            autoRight = true;
            nearZone();
            break;
        case    5:
            three = true;
            autoRight = false;
            nearZone();
            break;
        case    6:
            three = false;
            autoRight = true;
            nearZone();
            break;
        case    7:
            three = false;
            autoRight = false;
            nearZone();
            break;
        case    8:
            theFinalSolution();
            break;
        default:
            break;
        }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
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

//LINEAR DRIVE VARIABLES
int speedLeft;
int speedRight;

//Linear Drive Function
void setSpeed(){
    if(speedLeft > 127) speedLeft = 127;
    if(speedLeft < -127) speedLeft = -127;
		if(speedRight > 127) speedRight = 127;
    if(speedRight < -127) speedRight = -127;
    int absSpeedL = trueSpeed[abs(speedLeft)]*(speedLeft/abs(speedLeft+0.0001));
		int absSpeedR = trueSpeed[abs(speedRight)]*(speedRight/abs(speedRight+0.0001));
    	driveL(absSpeedL);
    	driveR(absSpeedR);
}

task partner(){
	while(true){
		speedLeft = vexRT[Ch3Xmtr2] + vexRT[Ch4Xmtr2] + vexRT[Ch1Xmtr2] ;

		speedRight = vexRT[Ch3Xmtr2] -  vexRT[Ch4Xmtr2] - vexRT[Ch1Xmtr2];
		if(abs(speedLeft)<20){
			speedLeft = 0;
			speedRight = 0;
		}
		setSpeed();
		motor[mogo] = (vexRT(Btn6DXmtr2)*137) - (vexRT[Btn6UXmtr2]*127) + 10;
	}
}
task solo(){
	while(true){
		speedLeft = vexRT[Ch3] + vexRT[Ch4];
		speedRight = vexRT[Ch3] -  vexRT[Ch4];
		if(abs(speedLeft)<20){
			speedLeft = 0;
			speedRight = 0;
		}
		setSpeed();
		motor[mogo] = (vexRT(Btn8U)*137) - (vexRT[Btn8L]*127) + 10;
	}
}

task usercontrol()
{
  startTask(partner);
	while(true)
	{
		motor[rollers] = (vexRT[Btn5U]*127) - (vexRT[Btn5D]*127) - 30;
		if(vexRT[Btn7L]){
			stopTask(partner);
			startTask(solo);
		}
		if(vexRT[Btn7R]){
			startTask(partner);
			stopTask(solo);
		}
		if(vexRT[Btn8R]){
				if(SensorValue(manipulator) > 100){
					translational(-127);
				}
				else{
					translational(-10);
				}
		}
		else if(abs(vexRT(Ch2)) > 10){
			translational(vexRT[Ch2]);
		}

		else if(vexRT[Btn8D]){
			if(SensorValue(manipulator) < 2100){
				translational(127);
			}
			else{
				translational(10);
			}
		}
		else{
			translational(0);
		}

		motor[liftL] = (vexRT(Btn6U)*127) - (vexRT[Btn6D]*127);
		motor[liftR] = (vexRT(Btn6U)*127) - (vexRT[Btn6D]*127);;
	}
}
