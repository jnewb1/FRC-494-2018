#include "WPILib.h"
#include "ctre/Phoenix.h"

#include "Timer.h"
#include "base.h"
#include "iTimer.h"

#include "ADIS16448_IMU.h"

void setDriveTrainRampRate(float t);





int teamNum = 70;

void setupValues();

void EngageAntiBackDrive();
void DegageAntiBackDrive();

float cam1DriveMode = 1;

int inAtonRotate = 0;

float curTime2;
float lastTime2 = 0;

float encoderCountsPerInch = -393;

float liftCountsPerInch = -393;

float toolbarCountsPerDegree = 100;

extern float PegGoalAngle;

void UpdateEncoders();

void ReleaseHooks();
void DetainHooks();
void Set_LiftHeight(float height);
void setTalonsToCoast();
void setTalonsToBrake();
void Set_AntiBackDrive(float angle);
void Set_HookRelease(float angle);
float lastSetPos = 0;
//#include <taskLib.h>
//void aton_move(float speed, float strafeSpeed, float time, float x = 0);
extern float inAton;
extern float inTele;
extern float inDisable;
float hopperSpeed = 0;
extern float sendBackValues[420];

void ReadEncoderSlow();

void getGameData();
extern std::string gameData;

float currentHopperPOS;

int joyOverride = 0;
float joyXOverride = 0;
float joyYOverride = 0;
float joyZOverride = 0;

void Set_Winch(float speed);
void SetAnalogEncoders();

int inTest = 0;

int leftFrontDrive1 = 13;
int leftFrontDrive2 = 11;
int rightFrontDrive1 = 2;
int rightFrontDrive2 = 4;
int leftBackDrive1 = 12;
int leftBackDrive2 = 10;
int rightBackDrive1 = 3;
int rightBackDrive2 = 5;

int gripperMotor = 6;
int spareMotor2 = 7;
int spareMotor3 = 15;
int winchMotor1 = 16;
int winchMotor2 = 1;
int toolbarMotor = 14;
int cubeLeftRoller = 9;
int cubeRightRoller = 8;

int turret = 17;

float sendVars[400];

void IFC_Local_Loop_Service(void);
void UpdateVariables();

void UDCReceiverTest();
void UDCReceiverTestfrompi();
int sendUDC(char *buf, int size);
int sendUDCtoPI(char *buf, int size);

void customJoySticks();

void setupWPI_TalonSRXs();

float gyroZeroY;

/*
 TalonSRX * hopperCAN;
 TalonSRX * shooterCAN;
 TalonSRX * winch1CAN;
 TalonSRX * winch2CAN;
 WTalonSRX* intakeRollerCAN;
 TalonSRX * toolbarCAN;
 TalonSRX * gearRightRollerCAN;
 TalonSRX * gearLeftRollerCAN;

 TalonSRX * rightFrontDrive1Motor;
 TalonSRX * rightFrontDrive2Motor;
 TalonSRX * leftFrontDrive1Motor;
 TalonSRX * leftFrontDrive2Motor;
 TalonSRX * rightBackDrive1Motor;
 TalonSRX * rightBackDrive2Motor;
 TalonSRX * leftBackDrive1Motor;
 TalonSRX * leftBackDrive2Motor;
 */

Joystick *m_driveStick;
Joystick *m_driveStick2;

float shooterLastSpeed = 0;
double shooterLastSpeedTime = 0;

//Servo hangServo;

ADIS16448_IMU *imu;

void Set_Motor() {

}

void Spit() {
}
void MecanumDrive(double x, double y, double rotation, double gyroAngle);
void mainRobotLoop();

void StartNewCameraCmd() {
	// GetMyIPAddress ();
	//	if (CameraConnectRetry+5.0>GetTime()) return;

	//#ifdef TEAM14

	// Camera = new iAxisCamera("10.0.14.11");

	//#endif

	//#ifdef TEAM70

	//Camera = new iAxisCamera("10.0.70.11");

	//#endiff

	//#ifdef TEAM494

	//Camera = new iAxisCamera("10.4.94.11");

	//#endif

	//	Camera = new iAxisCamera(TeamCameraIP);

}

int isEnabled() {

	return 1;

}

void MoveElev(float dest) {
	// ElevDest=dest;
	//ElevAuto=1;
}

void MoveShoe(float dest) {
	// ShoeDest=dest;
	//ShoeAuto=1;
}

void MoveToolBar(float dest) {
	// ToolBarDest=dest;
	//ToolBarAuto=1;
}

void SetSolenoid(int num, int state) {

	// if (num<0 || num>7) return;

	// if (state) solenoid[num]->Set(true);

	// else

	// solenoid[num]->Set(false);

}

extern float realGyroAngle;
/*
 void resetGyro()
 {
 //gyroZero=realGyroAngle;
 }
 */
void startTimer(uc num, int count, void (*serviceAddress)(uc),
		void (*finishAddress)(uc));

std::thread FastThread;

//char TeamCameraIP[20] = "10.4.94.11";

char TeamDriveStationIP[20] = "10.0.14.5";
char TeamPiIP[20] = "10.0.14.100";

float realGyroAngle = 0;
#define noCan1X
#define noCan2

#define max(x, y) (((x) > (y)) ? (x) : (y))

Solenoid *solenoid[20];

int operatorOnce = 0;

float gyroZero = 0;

float gyroAngle;

float gyroZeroField;

int gyroZeroFlag = 0;

int gTimeOut = 0;

void GyroOff() {

	//gyroZeroFlag = 1;
}

int bActive = 0;
int tActive = 0;
int aActive = 0;
int eActive = 0;
int sActive = 0;

Joystick *jStick1 = 0;
Joystick *jStick2 = 0;
//Joystick *jStick3=0;
//Joystick *jStick4=0;

float autoDriveY = 0.0;
float autoDriveX = 0.0;
float autoDriveR = 0.0;

int scanFlag = 0;

void userNewDriverStationPacket();
void userInit();

static float fabs(float val) {
	if (val >= 0)
		return val;

	return -val;
}

class MecanumDefaultCode: public IterativeRobot {
	//TalonSRX lf; /*left front */
	//TalonSRX lr;/*left rear */
	//TalonSRX rf; /*right front */
	//TalonSRX rr; /*right rear */

	TalonSRX * _talon20 = new TalonSRX(20);
	TalonSRX * _talon21 = new TalonSRX(21);
	TalonSRX * _talon22 = new TalonSRX(22);
	TalonSRX * _talon23 = new TalonSRX(23);

public:

	Ultrasonic *ultra1 = new Ultrasonic(0,1);
	Ultrasonic *ultra2 = new Ultrasonic(2,3);
	AnalogInput *ai = new AnalogInput(0);
	Potentiometer *pot = new AnalogPotentiometer(ai, 10000, 30);
	Servo *antibackdrive = new Servo(0);
	Servo *hook1 = new Servo(1);
	Servo *hook2 = new Servo(2);

	TalonSRX * gripperCAN = new TalonSRX(gripperMotor);
	TalonSRX * spare2CAN = new TalonSRX(spareMotor2);
	TalonSRX * spare3CAN = new TalonSRX(spareMotor3);

	TalonSRX * winch1CAN = new TalonSRX(winchMotor1);
	TalonSRX * winch2CAN = new TalonSRX(winchMotor2);

	TalonSRX * toolbarCAN = new TalonSRX(toolbarMotor);
	TalonSRX * rightRollerCAN = new TalonSRX(cubeRightRoller);
	TalonSRX * leftRollerCAN = new TalonSRX(cubeLeftRoller);

	TalonSRX * rightFrontDrive1Motor = new TalonSRX(rightFrontDrive1);
	TalonSRX * rightFrontDrive2Motor = new TalonSRX(rightFrontDrive2);
	TalonSRX * leftFrontDrive1Motor = new TalonSRX(leftFrontDrive1);
	TalonSRX * leftFrontDrive2Motor = new TalonSRX(leftFrontDrive2);
	TalonSRX * rightBackDrive1Motor = new TalonSRX(rightBackDrive1);
	TalonSRX * rightBackDrive2Motor = new TalonSRX(rightBackDrive2);
	TalonSRX * leftBackDrive1Motor = new TalonSRX(leftBackDrive1);
	TalonSRX * leftBackDrive2Motor = new TalonSRX(leftBackDrive2);
	PowerDistributionPanel * pdp = new PowerDistributionPanel(0);
	//RobotDrive *m_robotDrive;		// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_driveStick;	// Joystick object on USB port 1 (mecanum drive)public:
	Joystick *m_shootStick;
	Joystick *m_driveStick2;


	//AnalogGyro gyro;
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */

	MecanumDefaultCode(void) //: //gyro(0) //lf(20), lr(21), rf(22), rr(23), gyro(0)
			{

		imu = new ADIS16448_IMU;
		printf("MecanumDefaultCode\n");
		userInit();

		m_driveStick = new Joystick(0);
		m_shootStick = new Joystick(1);

	}
	void RotateVector(double &x, double &y, double angle) {
		double cosA = cos(angle * (3.14159 / 180.0));
		double sinA = sin(angle * (3.14159 / 180.0));
		double xOut = x * cosA - y * sinA;
		double yOut = x * sinA + y * cosA;
		x = xOut;
		y = yOut;
	}
	void Normalize(double *wheelSpeeds) {
		double maxMagnitude = fabs(wheelSpeeds[0]);
		int32_t i;
		for (i = 1; i < 4; i++) {
			double temp = fabs(wheelSpeeds[i]);
			if (temp > maxMagnitude)
				maxMagnitude = temp;
		}
		if (maxMagnitude > 1.0) {
			for (i = 0; i < 4; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
			}
		}
	}

	//I2C *lidar = new I2C(I2C::Port::kOnboard, 0x62);
	/*
	 void readyUp()
	 {
	 const int tries = 5;
	 int ret = 1;
	 for (int i = 0; ret != 0 && i < tries; i++)
	 {
	 ret = lidar->Write(0x00, 0x04);

	 ::Wait(0.004);
	 }
	 }

	 double getDistance() //returns distance in centimeters
	 {
	 const int tries = 10;
	 int ret = 1;

	 readyUp();
	 unsigned char *distArray = new unsigned char[8];
	 for (int i = 0; ret != 0 && i < tries; i++)
	 {
	 ::Wait(0.003);
	 ret = lidar->Read(0x8f, 2, distArray);
	 //printf("Success? %d\n", ret);
	 }

	 int centimeters = (distArray[0] << 8) + distArray[1];

	 delete[] distArray;

	 return centimeters;
	 return 0;
	 }
	 */

	float vars[2000];

	void mainRobotLoop() {


		//vars[0] = selectCamera;

		//sendUDCtoPI((char*) vars, 8000);

		customJoySticks();

		userNewDriverStationPacket();

		IFC_Local_Loop_Service();

		UDCReceiverTest();

		UDCReceiverTestfrompi();

		GlobalSendBack();

		char *ptr = (char*) &sendVars[0];

		ptr[0] = 1;
		ptr[1] = 19;
		ptr[2] = 2;
		ptr[3] = 108;

		for (int i = 1; i < 400; i++) {
			sendVars[i] = sendBackValues[i + 1];
		}

		sendUDC((char*) sendVars, 1600);

		static double lastTime3 = 0;
		static double lastTime4 = 0;

		double curTime = Timer::GetFPGATimestamp();

		double dif2 = curTime - lastTime3;
		double dif3 = curTime - lastTime4;


		if (dif2 < 0.01)
			return;

		if (dif3 > .1) {
			UpdateVariables();
			lastTime4 = curTime;
		}

		lastTime3 = curTime;

		double gyroX = imu->GetAngleX();

		GyroX = imu->GetAngleX();
		GyroY = imu->GetAngleY();
		GyroZ = imu->GetAngleZ();


		double joyXX = m_driveStick->GetRawAxis(0); //if ((joyX)<0.01) joyX=0; if (joyX>limit) joyX=limit;
		double joyYY = m_driveStick->GetRawAxis(1); //if ((joyY)<0.01) joyY=0; if (joyY>limit) joyY=limit;
		double joyZZ = m_driveStick->GetRawAxis(4); //if ((joyZ)<0.01) joyZ=0; if (joyZ>limit) joyZ=limit;

		if (fabs(joyXX + joyYY + joyZZ) > 0.2) {
			joyOverride = 0;
		}
		float rightVel;
		float leftVel;
		if (joyOverride) {
			//printf("OVERRIDE %3.3f %3.3f %3.3f\n", joyXOverride, joyYOverride,
					//joyZOverride);
			joyX[2] = joyXOverride;
			joyY[2] = joyYOverride;
			joyX2[2] = joyZOverride;
		}
		else {
			joyX[2] = joyXX;
			joyY[2] = joyYY;
			joyX2[2] = joyZZ;
		}
		if (joyOverride==2){
			rightVel=joyYOverride;
			leftVel=joyYOverride;
		}


		gyroAngle = -gyroX;

		//gyroAngle*= TuningVar5;

		float x = joyX[2];
		if (x < 0.05 && x > -0.05)
			x = 0;
		float y = joyY[2];
		if (y < 0.05 && y > -0.05)
			y = 0;
		float r = joyX2[2];
		if (r < 0.05 && r > -0.05)
			r = 0;

		static float lastx, lasty, lastr;
		static int gTimeOut = 0;
		static double releaseTime = 0;

		if (x == lastx && y == lasty && r == lastr && inAton == 0) {
			gTimeOut++;

			if (gTimeOut > 500)
				gyroZero = gyroAngle;

		} else
			gTimeOut = 0;

		lastx = x;
		lasty = y;
		lastr = r;

		if (gyroZeroFlag && inAton == 0) {
			gyroZero = gyroAngle;
			gyroZeroFlag = 0;
		}

		if (fabs(r) > 0.2 && inAton == 0) {
			releaseTime = Timer::GetFPGATimestamp() + 0.5;

			gyroZero = gyroAngle;
		}

		if (Timer::GetFPGATimestamp() < releaseTime && inAton == 0) {
			gyroZero = gyroAngle;
		}

		if(fabs(y)>0.5){
			gyroZeroY = GyroY;
		}

		if (!inAton){
		x = x * x * x;

		y = y * y * y;

		r *= 0.8;

		r = r * r * r;
		}
		float gyroErr = (gyroAngle - gyroZero);
		float gyroAdj = gyroErr/40;


		float gyroAdjY = 0;
		float yError = (GyroY - gyroZeroY);

		if(fabs(yError)>20){
			gyroAdjY = yError / -20;
			//gyroAdjY = 0;
			//Set_LiftHeight(0);
			//Set_ToolBar(50);
		}

		if (inAtonRotate==1) {
			gyroAdj = 0;
		}
		if (inAtonRotate==2) {
			gyroAdj = gyroAdj*5;
		}


		r = r - gyroAdj;
		y = y - gyroAdjY;

		double xIn = x;
		double yIn = y;
		double rotation = r;
		TuningVar5 = r;
		yIn = -yIn;

		RotateVector(xIn, yIn, 0);

		gyro_Zero = gyroZero;
		GyroLoc = gyroAngle - gyroZero;

		double wheelSpeeds[4];
		wheelSpeeds[1] = -(xIn + yIn + rotation);
		wheelSpeeds[2] = (-xIn + yIn - rotation);
		wheelSpeeds[3] = -(-xIn + yIn + rotation);
		wheelSpeeds[4] = (xIn + yIn - rotation);

		Normalize(wheelSpeeds);
		leftFrontDrive1Motor->Set(ControlMode::PercentOutput, wheelSpeeds[1]);
		leftFrontDrive2Motor->Set(ControlMode::PercentOutput, wheelSpeeds[1]);
		rightFrontDrive1Motor->Set(ControlMode::PercentOutput, wheelSpeeds[2]);
		rightFrontDrive2Motor->Set(ControlMode::PercentOutput, wheelSpeeds[2]);
		leftBackDrive1Motor->Set(ControlMode::PercentOutput, wheelSpeeds[3]);
		leftBackDrive2Motor->Set(ControlMode::PercentOutput, wheelSpeeds[3]);
		rightBackDrive1Motor->Set(ControlMode::PercentOutput, wheelSpeeds[4]);
		rightBackDrive2Motor->Set(ControlMode::PercentOutput, wheelSpeeds[4]);

			/*
			leftFrontDrive1Motor->Set(ControlMode::Velocity, leftVel);
			leftFrontDrive2Motor->Set(ControlMode::Velocity, leftVel);
			leftBackDrive1Motor->Set(ControlMode::Velocity, leftVel);
			leftBackDrive2Motor->Set(ControlMode::Velocity, leftVel);
			rightFrontDrive1Motor->Set(ControlMode::Velocity, rightVel);
			rightFrontDrive2Motor->Set(ControlMode::Velocity, rightVel);
			rightBackDrive1Motor->Set(ControlMode::Velocity, rightVel);
			rightBackDrive2Motor->Set(ControlMode::Velocity, rightVel);
			*/
	}

	void RobotInit() {
		ultra1->SetAutomaticMode(true);
		ultra2->SetAutomaticMode(true);
		matchStatus = 3;
		DegageAntiBackDrive();
		DetainHooks();
		EncoderGain = -273;
		ToolBarGain = 58.6;
		LiftGain = -128.5;

		favorScale=1;
		Crossover=1;
		TwoCube=1;
		HoldCube=0;

		//matchNumber = frc::DriverStation::GetInstance().GetMatchNumber();
		printf("RobotInit\n");


		if (teamNum == 70) {
			strcpy(TeamDriveStationIP, "10.0.70.5");
			strcpy(TeamPiIP, "10.0.70.100");
		}
		if (teamNum == 494) {
			strcpy(TeamDriveStationIP, "10.4.94.5");
			strcpy(TeamPiIP, "10.4.94.199");
		}

		if (teamNum == 14) {

			strcpy(TeamDriveStationIP, "10.0.14.5");
			strcpy(TeamPiIP, "10.0.14.100");

		}

		setupWPI_TalonSRXs();
		setupValues();

	}
	void zeroGyro() {
		gyroZero = gyroAngle;
	}
	void zeroGyroY(){
		gyroZeroY = GyroY;
	}

	void DisabledInit() {
		matchStatus = 3;
		printf("DisableInit\n");
		stopTimer(19);
		joyOverride = 0;
		mainRobotLoop();
	}
	void DisabledPeriodic() {
		matchStatus = 3;
		getGameData();
		joyOverride = 0;
		inDisable = 1;
		zeroGyro();
		zeroGyroY();
		inTest = 0;
		inTele = 0;
		inAton = 0;
		inAtonRotate = 0;
		mainRobotLoop();
	}
	void RobotPeriodic() {
		mainRobotLoop();
	}
	void TeleopInit() {
		matchStatus = 2;
		setDriveTrainRampRate(0.1);
		inAtonRotate = 0;
		joyOverride = 0;
		printf("TeleopInit\n");
		setTalonsToBrake();
		mainRobotLoop();
	}
	void AutonomousInit() {
		matchStatus = 1;
		DegageAntiBackDrive();
		DetainHooks();
		setDriveTrainRampRate(0.1);
		gyroZeroField = gyroAngle;
		printf("AutomousInit\n");
		setTalonsToBrake();
		mainRobotLoop();
	}
	void AutonomousPeriodic() {
		matchStatus = 1;
		inDisable = 0;
		inTest = 0;
		inTele = 0;
		inAton = 1;
		mainRobotLoop();
	}
	void TestInit() {
		printf("TestInit\n");
		mainRobotLoop();
	}
	void TestPeriodic() {
		inDisable = 0;
		inTest = 1;
		inTele = 0;
		inAton = 0;
		inAtonRotate = 0;
		mainRobotLoop();
	}
	//void StartCompetition  (){printf("StartCompetition\n");}

	/** @return 10% deadband */
	double Db(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void) {
		stopTimer(19);
		matchStatus = 2;
		inAtonRotate = 0;
		mainRobotLoop();
	}
};

MecanumDefaultCode * robott;

#define START_ROBOT_CLASSx(_ClassName_)                                       \
  int main() {                                                               \
    if (!HAL_Initialize(500,0)) {                                                \
        llvm::errs() << "FATAL ERROR: HAL could not be initialized\n";    \
      return -1;                                                             \
    }                                                                        \
    HAL_Report(HALUsageReporting::kResourceType_Language,                    \
               HALUsageReporting::kLanguage_CPlusPlus);                      \
    static _ClassName_ robot;                                                \
    robott=&robot; \
    llvm::outs() << "\n********** Robot program starting **********\n"; \
    robot.StartCompetition();                                                \
  }

START_ROBOT_CLASSx(MecanumDefaultCode);

//static MecanumDefaultCode robot;

void setupValues(){
	if(fabs(ToolBarPickup)<10) ToolBarPickup=85;
	if(fabs(ToolBarSwitch)<10) ToolBarSwitch=60;
	if(fabs(ToolBarScaleLow)<10) ToolBarScaleLow=50;
	if(fabs(ToolBarScaleMid)<10) ToolBarScaleMid=50;
	if(fabs(ToolBarScaleHigh)<10) ToolBarScaleHigh=40;

	if(fabs(LiftLow)>20) LiftLow=1;
	if(fabs(LiftSwitch)<10) LiftSwitch=30;
	if(fabs(LiftScaleLow)<10) LiftScaleLow=60;
	if(fabs(LiftScaleMid)<10) LiftScaleMid=78;
	if(fabs(LiftScaleHigh)<10) LiftScaleHigh=85;

	if(fabs(LiftScaleBehind)<10) LiftScaleBehind=85;

	if(fabs(ToolBarClimb)<10) ToolBarClimb=44;
	if(fabs(LiftClimb)<10) LiftClimb=82;

	if(fabs(ToolBarClimbSide)<0) ToolBarClimbSide=5;
	if(fabs(LiftClimbSide)<0) LiftClimbSide=74;

	if(fabs(move_ApproachSwitch)<10) move_ApproachSwitch = 50;
	if(fabs(move_CenterofSwitch)<10) move_CenterofSwitch = 100; //aton1
	if(fabs(move_TouchSwitch)<10) move_TouchSwitch    = 15;

	if(fabs(move_SideCenterofSwitch)<10) move_SideCenterofSwitch = 235;
	if(fabs(move_BehindSwitch)<10) move_BehindSwitch = 335;        //aton2 and 3
	if(fabs(move_SwitchfromBehind)<10) move_SwitchfromBehind = 250;

	if(fabs(move_FaceScale)<10) move_FaceScale = 405; //435
	if(fabs(move_ScalefromBehind)<10) move_ScalefromBehind = 282;

	if(fabs(move_CrossDropCube)<5) move_CrossDropCube=60; //435
	if(fabs(move_CornerDropCube)<5) move_CornerDropCube=30;

	if(fabs(move_TwoCubeDistance)<5) move_TwoCubeDistance=75;
	if(fabs(move_CornerToScale)<10) move_CornerToScale=340;
	if(fabs(TwoCubePickupAngle)<20) TwoCubePickupAngle=80;
	if(fabs(move_OverHeadDroppoff)<20) move_OverHeadDroppoff=60;
}

void setTalonToDefault(TalonSRX* talon) {
	talon->SetNeutralMode(NeutralMode::Brake);
	talon->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	talon->ConfigNominalOutputForward(0, 0);
	talon->ConfigNominalOutputReverse(0, 0);
	talon->ConfigPeakOutputForward(1, 0);
	talon->ConfigPeakOutputReverse(-1, 0);

	talon->SetInverted(false);
	talon->SetSensorPhase(false);
	talon->EnableCurrentLimit(false);
	talon->EnableVoltageCompensation(true);
}

void setTalonToName(TalonSRX* talon, const llvm::Twine & name) {
}
void setTalonToBrake(TalonSRX* talon) {
	talon->SetNeutralMode(NeutralMode::Brake);
}
void setTalonMaxAmps(TalonSRX* talon, float amps) {
	talon->ConfigContinuousCurrentLimit(int(amps), 10);
	//talon->EnableCurrentLimit(true);
}
void setTalonPeakAmps(TalonSRX* talon, float amps) {
	talon->ConfigPeakCurrentLimit(int(amps), 10);
}
void setTalonToCoast(TalonSRX* talon) {
	talon->SetNeutralMode(NeutralMode::Coast);
}

void setTalonPID(TalonSRX* talon, float p, float i, float d) {
	talon->Config_kP(0, p, 0);
	talon->Config_kI(0, i, 0);
	talon->Config_kD(0, d, 0);
	talon->Config_kF(0, 0, 0);
}
void setTalonRemoteEncoder(TalonSRX* talon, int remote){
	double kP = 0.05;
	double kI = 0.01;
	double kD = 0;
	talon->ConfigRemoteFeedbackFilter(remote,RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,0,10);
	setTalonPID(talon, kP, kI, kD);
}

void reverseTalon(TalonSRX* talon){
	talon->SetInverted(true);
}
void reverseTalonEncoder(TalonSRX* talon){
	talon->SetSensorPhase(true);
}

void setTalonRampRate(TalonSRX* talon, float t)
{
	talon->ConfigOpenloopRamp(t, 0);
}

void setTalonMaxSpeed(TalonSRX* talon, float s, float rs=-90){
	if(rs==-90){
		talon->ConfigPeakOutputForward(s, 0);
		talon->ConfigPeakOutputReverse(-s, 0);
	}
	else{
		talon->ConfigPeakOutputForward(s, 0);
		talon->ConfigPeakOutputReverse(-rs, 0);
	}
}

void setDriveTrainRampRate(float t){
	setTalonRampRate(robott->rightFrontDrive1Motor, t);
	setTalonRampRate(robott->rightFrontDrive2Motor, t);
	setTalonRampRate(robott->leftFrontDrive1Motor, t);
	setTalonRampRate(robott->leftFrontDrive2Motor, t);
	setTalonRampRate(robott->rightBackDrive1Motor, t);
	setTalonRampRate(robott->rightBackDrive2Motor, t);
	setTalonRampRate(robott->leftBackDrive1Motor, t);
	setTalonRampRate(robott->leftBackDrive2Motor, t);
}
void setupWPI_TalonSRXs() {
	printf("setting up WPI_TalonSRXs");


	setTalonToDefault(robott->rightFrontDrive1Motor); //make all motors set to default.
	setTalonToDefault(robott->rightFrontDrive2Motor); //they save their settings across firmware updates!
	setTalonToDefault(robott->leftFrontDrive1Motor);
	setTalonToDefault(robott->leftFrontDrive2Motor);
	setTalonToDefault(robott->rightBackDrive1Motor);
	setTalonToDefault(robott->rightBackDrive2Motor);
	setTalonToDefault(robott->leftBackDrive1Motor);
	setTalonToDefault(robott->leftBackDrive2Motor);

	//setDriveTrainRampRate(0);

//	setTalonToDefault(robott->toolbarCAN);
	setTalonToDefault(robott->leftRollerCAN);
	setTalonToDefault(robott->rightRollerCAN);
//	setTalonToDefault(robott->winch1CAN);
//	setTalonToDefault(robott->winch2CAN);

	setTalonRampRate(robott->toolbarCAN, 0.5);
	setTalonRampRate(robott->winch1CAN, 0.5);
	setTalonRampRate(robott->winch2CAN, 0.5);

	setTalonMaxSpeed(robott->toolbarCAN, 0.5, 0.7); //0.4, 0.5

	/*
	setTalonPeakAmps(robott->winch1CAN, 60);
	setTalonPeakAmps(robott->winch2CAN, 60);
	setTalonMaxAmps(robott->winch1CAN, 40);
	setTalonMaxAmps(robott->winch2CAN, 40);

*/
//	setTalonMaxAmps(robott->toolbarCAN, 30);
//	setTalonPeakAmps(robott->toolbarCAN, 40);

	robott->leftFrontDrive1Motor->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20, 0);
	setTalonPID(robott->toolbarCAN, 3, 0,0);
	setTalonPID(robott->winch1CAN, 0.5, 0.0, 0.00);

	reverseTalon(robott->toolbarCAN);
	if(teamNum==70){
		reverseTalonEncoder(robott->toolbarCAN);
	}

	reverseTalon(robott->winch1CAN);
	reverseTalon(robott->winch2CAN);
	reverseTalonEncoder(robott->winch1CAN);
}

void setTalonsToCoast() {
	setTalonToCoast(robott->rightFrontDrive1Motor);
	setTalonToCoast(robott->rightFrontDrive2Motor);
	setTalonToCoast(robott->leftFrontDrive1Motor);
	setTalonToCoast(robott->leftFrontDrive2Motor);
	setTalonToCoast(robott->rightBackDrive1Motor);
	setTalonToCoast(robott->rightBackDrive2Motor);
	setTalonToCoast(robott->leftBackDrive1Motor);
	setTalonToCoast(robott->leftBackDrive2Motor);
}
void setTalonsToBrake() {
	setTalonToBrake(robott->rightFrontDrive1Motor);
	setTalonToBrake(robott->rightFrontDrive2Motor);
	setTalonToBrake(robott->leftFrontDrive1Motor);
	setTalonToBrake(robott->leftFrontDrive2Motor);
	setTalonToBrake(robott->rightBackDrive1Motor);
	setTalonToBrake(robott->rightBackDrive2Motor);
	setTalonToBrake(robott->leftBackDrive1Motor);
	setTalonToBrake(robott->leftBackDrive2Motor);
}

float wheelZero = 0;


void enableStrafeUltrasonic(){
	robott->ultra1->SetEnabled(true);
}
void enableCubeUltrasonic(){
	robott->ultra2->SetEnabled(true);
}
void disableStrafeUltrasonic(){
	robott->ultra1->SetEnabled(false);
}
void disableCubeUltrasonic(){
	robott->ultra2->SetEnabled(false);
}
bool isCubeUltrasonicValid(){
	return robott->ultra2->IsRangeValid();
}
float Get_ToolbarError() {
	return (robott->toolbarCAN->GetClosedLoopError(0));
}
float Get_LiftError() {
	return (robott->winch1CAN->GetClosedLoopError(0));
}
float Get_DistanceEncoderInches() {
	return ((robott->leftFrontDrive1Motor->GetSelectedSensorPosition(0))
			/ EncoderGain);
}
float Get_UltrasonicInches() {
	return robott->ultra1->GetRangeInches();
}
float Get_Ultrasonic2Inches() {
	return robott->ultra2->GetRangeInches();
}
float Get_DistanceEncoder() {
	return (robott->leftFrontDrive1Motor->GetSelectedSensorPosition(0));
}
float Get_DistanceEncoderVelocity() {
	return (robott->leftFrontDrive1Motor->GetSelectedSensorVelocity(0))
			/ (EncoderGain / 10);
}

float Get_LiftHeight() {
	return robott->winch1CAN->GetSelectedSensorPosition(0) / LiftGain;
}
float Get_LiftSetPoint() {
	return robott->winch1CAN->GetClosedLoopTarget(0) / LiftGain;
}

float Get_ToolBarAngle() {
	return robott->toolbarCAN->GetSelectedSensorPosition(0) / ToolBarGain;
}
float Get_ToolBarSetPoint() {
	return robott->toolbarCAN->GetClosedLoopTarget(0) / ToolBarGain;
}
float Get_ShooterY() {
	return robott->m_shootStick->GetRawAxis(1);
}
void Set_CubeIntake(float speed1, float speed2) {
	robott->leftRollerCAN->Set(ControlMode::PercentOutput, -speed1);
	robott->rightRollerCAN->Set(ControlMode::PercentOutput, speed2);
}

void Set_Lift(float speed) {
	robott->winch1CAN->Set(ControlMode::PercentOutput, speed);
	robott->winch2CAN->Set(ControlMode::PercentOutput, speed);
}

void Set_Gripper(float speed){
	robott->gripperCAN->Set(ControlMode::PercentOutput, -speed);
}
//winch2 is a follower of winch1

void Set_ToolBar(float speed) {
	robott->toolbarCAN->Set(ControlMode::PercentOutput, speed);
}

void Set_LiftHeight(float height) {
	robott->winch2CAN->Set(ControlMode::Follower, winchMotor1);
	robott->winch1CAN->Set(ControlMode::Position, height * LiftGain);
}
void Set_ToolBarAngle(float angle) {
	robott->toolbarCAN->Set(ControlMode::Position, angle * ToolBarGain);
}

void Set_AntiBackDrive(float angle){
	robott->antibackdrive->SetAngle(angle);
}
void Set_HookRelease(float angle){
	robott->hook1->SetAngle(angle);
}

void zeroToolBar(){

	robott->toolbarCAN->SetSelectedSensorPosition(0, 0, 10);
	Set_ToolBarAngle(0);
}
void zeroLift(){
	robott->winch1CAN->SetSelectedSensorPosition(0, 0, 10);
	Set_LiftHeight(0);
}

void switchCam() {

	if (selectCamera == 1) {
		printf("switching camera2\n");
		selectCamera = 2;
	} else if (selectCamera == 2) {
		printf("switching camera1\n");
		selectCamera = 1;
	}
}
void switchDriveMode() {

	if (cam1DriveMode == 1) {
		printf("switching camera drive mode 2\n");
		cam1DriveMode = 2;
	} else if (cam1DriveMode == 2) {
		printf("switching camera drive mode 1\n");
		cam1DriveMode = 1;
	}
}

void UpdateVariables() {
	matchTime = frc::DriverStation::GetInstance().GetMatchTime();


	//printf("UpdateVariables\n");
	//matchTimeLeft = frc::DriverStation::GetInstance().GetMatchTime();


	EncoderDistance = Get_DistanceEncoderInches();
	EncoderVelocity = Get_DistanceEncoderVelocity();
	BatteryVoltage = robott->pdp->GetVoltage();

	ultra1Distance = robott->ultra1->GetRangeInches();
	ultra2Distance = robott->ultra2->GetRangeInches();

	//robotWatts = robott->pdp->GetTotalPower();
	//TuningVar5 = Get_ToolbarError();
	//TuningVar4 = Get_ToolbarError()*ToolBarGain;

	RBD2Speed = robott->rightBackDrive2Motor->GetMotorOutputPercent();
	RBD1Speed = robott->rightBackDrive1Motor->GetMotorOutputPercent();
	LBD2Speed = robott->leftBackDrive2Motor->GetMotorOutputPercent();
	LBD1Speed = robott->leftBackDrive1Motor->GetMotorOutputPercent();
	RFD2Speed = robott->rightFrontDrive2Motor->GetMotorOutputPercent();
	RFD1Speed = robott->rightFrontDrive1Motor->GetMotorOutputPercent();
	LFD2Speed = robott->leftFrontDrive2Motor->GetMotorOutputPercent();
	LFD1Speed = robott->leftFrontDrive1Motor->GetMotorOutputPercent();
	RBD2Amps = robott->rightBackDrive2Motor->GetOutputCurrent();
	RBD1Amps = robott->rightBackDrive1Motor->GetOutputCurrent();
	LBD2Amps = robott->leftBackDrive2Motor->GetOutputCurrent();
	LBD1Amps = robott->leftBackDrive1Motor->GetOutputCurrent();
	RFD2Amps = robott->rightFrontDrive2Motor->GetOutputCurrent();
	RFD1Amps = robott->rightFrontDrive1Motor->GetOutputCurrent();
	LFD2Amps = robott->leftFrontDrive2Motor->GetOutputCurrent();
	LFD1Amps = robott->leftFrontDrive1Motor->GetOutputCurrent();

	IntakeRightSpeed = (robott->rightRollerCAN->GetMotorOutputPercent());
	IntakeRightVolts = (robott->rightRollerCAN->GetMotorOutputVoltage());
	IntakeRightAmps = (robott->rightRollerCAN->GetOutputCurrent());

	IntakeLeftSpeed = (robott->leftRollerCAN->GetMotorOutputPercent());
	IntakeLeftVolts = (robott->leftRollerCAN->GetMotorOutputVoltage());
	IntakeLeftAmps = (robott->leftRollerCAN->GetOutputCurrent());

	LiftDest = Get_LiftSetPoint();
	LiftHeight = Get_LiftHeight();

	Lift2Volts = robott->winch2CAN->GetMotorOutputVoltage();
	Lift1Volts = robott->winch2CAN->GetMotorOutputVoltage();
	Lift2Amps = robott->winch2CAN->GetOutputCurrent();
	Lift1Amps = robott->winch2CAN->GetOutputCurrent();
	Lift2Speed = robott->winch2CAN->GetMotorOutputPercent();
	Lift1Speed = robott->winch2CAN->GetMotorOutputPercent();



	ToolBarVolts = (robott->toolbarCAN->GetMotorOutputVoltage());
	ToolBarAmps = (robott->toolbarCAN->GetOutputCurrent());
	ToolBarDest = Get_ToolBarSetPoint();
	ToolBarAngle = Get_ToolBarAngle();
	ToolBarSpeed = robott->toolbarCAN->GetMotorOutputPercent();

	TuningVar1 = BoilerGoalX;
	TuningVar2 = BoilerGoalY;



}
