package frc.team555.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import org.montclairrobotics.sprocket.actions.Action;
import org.montclairrobotics.sprocket.actions.State;
import org.montclairrobotics.sprocket.actions.StateMachine;
import org.montclairrobotics.sprocket.auto.AutoMode;
import org.montclairrobotics.sprocket.auto.states.*;
import org.montclairrobotics.sprocket.control.*;
import org.montclairrobotics.sprocket.drive.DriveModule;
import org.montclairrobotics.sprocket.drive.DriveTrainBuilder;
import org.montclairrobotics.sprocket.drive.DriveTrainType;
import org.montclairrobotics.sprocket.drive.InvalidDriveTrainException;
import org.montclairrobotics.sprocket.drive.steps.*;
import org.montclairrobotics.sprocket.drive.utils.GyroLock;
import org.montclairrobotics.sprocket.frc.*;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Degrees;
import org.montclairrobotics.sprocket.geometry.Distance;
import org.montclairrobotics.sprocket.geometry.XY;
import org.montclairrobotics.sprocket.loop.Updater;
import org.montclairrobotics.sprocket.motors.SEncoder;
import org.montclairrobotics.sprocket.motors.Module.MotorInputType;
import org.montclairrobotics.sprocket.utils.Debug;
import org.montclairrobotics.sprocket.utils.Input;
import org.montclairrobotics.sprocket.utils.PID;
import org.montclairrobotics.sprocket.utils.ZeroInput;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends FRCRobot {

	public static enum GEAR_MODE {MANUAL,AUTO};
	public static GEAR_MODE gearMode=GEAR_MODE.MANUAL;
	
	private static final int IMG_WIDTH = 320,IMG_HEIGHT = 240;
	private static final int 
		DriveStickID=0,
		AuxStickID=1,
		CloseSwitchLeftID=0,//limit switches
		OpenSwitchLeftID=1,
		CloseSwitchRightID=7,//%Better comments
		OpenSwitchRightID=6,
		GearButtonID=1,
		GearButtonAutoID=7,
		GearButtonManualID=8,
		GyroLockOff=6,
		FieldCentricButtonID=2,
		GyroLockButtonID=7,
		ResetGyroID=3,
		//VisionButtonID=4,
		LeftButtonID=4,
		RightButtonID=5,
		ManualOpen1=9,
		ManualClose1=11,
		ManualOpen2=10,
		ManualClose2=12,
		FullSpinButtonID=10,
		ClimbFastID=5,
		ClimbSlowID=3;
	

	private static final Distance ENC_SPEED = new Distance(1);
	private static final double MAX_ENC_ACCEL = 13;
	private static final double MAX_ENC_TICKS = 25;

	protected static final double FULL_SPIN_SPEED = 1;
	protected static final double SLOW_SPIN_SPEED = 0.45;

	protected static final double SPINNY_POWER = 1;
	
	private static boolean lastAutoGear=true;
	
	private FRCJoystick driveStick;
	private FRCJoystick auxStick;
	
	private DriveTrainBuilder builder;
	//private DriveTrain driveTrain;
	
	private FRCMotor gearLeftMotor, gearRightMotor;
	private DigitalInput openLeftSwitch, closeLeftSwitch, openRightSwitch, closeRightSwitch;
	
	private FRCMotor ropeMotor1;
	private FRCMotor ropeMotor2;
	
	private FRCMotor ballSpinny;
	private FRCMotor ballShooty;
	
	private SEncoder encRight;
	private SEncoder encLeft;

	private NavXRollInput navX;
	
	private Gear gear;
	private DashboardInput gearSpeedInput;
	
	private PowerDistributionPanel pdp;
	
	private AccelLimit accelLimit;

	private double shootStartTime;

	
	@Override
	public void robotInit() {
		//Joysticks
		driveStick = new FRCJoystick(DriveStickID);
		auxStick = new FRCJoystick(AuxStickID);
		//Gear opened/closed limit switches
		openLeftSwitch = new DigitalInput(OpenSwitchLeftID);
		closeLeftSwitch = new DigitalInput(CloseSwitchLeftID);
		openRightSwitch = new DigitalInput(OpenSwitchRightID);
		closeRightSwitch = new DigitalInput(CloseSwitchRightID);
		
		//Setting up gear trigger
		gearLeftMotor = new FRCMotor(new VictorSP(0));
		gearLeftMotor.setInverted(true);
		gearRightMotor = new FRCMotor(new CANTalon(5));
		gearRightMotor.setInverted(true);
		gear = new Gear(gearLeftMotor,openLeftSwitch,closeLeftSwitch, gearRightMotor, openRightSwitch, closeRightSwitch);
		gearSpeedInput = new DashboardInput("gear speed", 0.5);
		
		//DRIVE STICK TO AUX STICK CHANGE
		FRCButton gearButton = new FRCButton(auxStick, GearButtonID);
		gearButton.setAction(new Action() {

			@Override
			public void start() {

			}

			@Override
			public void enabled() {
				if(gearMode == GEAR_MODE.AUTO)
				{
					gear.openLimit();
				}
			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {
				if(gearMode == GEAR_MODE.AUTO)
				{
					gear.closeLimit();
				}
			}
		});
		
		FRCButton manual1Open=new FRCButton(auxStick,ManualOpen1);
		FRCButton manual1Close=new FRCButton(auxStick,ManualClose1);
		
		FRCButton manual2Open = new FRCButton(auxStick, ManualOpen2);
		FRCButton manual2Close = new FRCButton(auxStick, ManualClose2);
		
		manual1Open.setAction(new Action(){
			@Override
			public void start() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.open1();
				}
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {
				if(gearMode == GEAR_MODE.MANUAL){
					gear.stop1();
				}
			}

			@Override
			public void disabled() {

			}
		});
		
		
		manual1Close.setAction(new Action(){
			@Override
			public void start() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.close1();
				}
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop1();
				}
			}

			@Override
			public void disabled() {

			}
		});
		
		manual2Open.setAction(new Action(){
			@Override
			public void start() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.open2();
				}
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop2();
				}
			}

			@Override
			public void disabled() {

			}
		});
		
		
		manual2Close.setAction(new Action(){
			@Override
			public void start() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.close2();
				}
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop2();
				}
			}

			@Override
			public void disabled() {

			}
		});

		
		FRCButton manualGearToggle = new FRCButton(auxStick, 8);
		manualGearToggle.setAction(new Action() {
			@Override
			public void start() {
				if(gearMode==GEAR_MODE.AUTO)
				{
					gearMode=GEAR_MODE.MANUAL;
				}
				else
				{
					gearMode=GEAR_MODE.AUTO;
				}
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		});
		
		
		
		
		//Rope climber motors
		/*ropeMotor1 = new ControlledMotor(new CANTalon(6), new JoystickYAxis(auxStick));
		ropeMotor1.constrain(0.0, 1.0);
		ropeMotor1.getMotor().setInverted(true);
		ropeMotor2 = new ControlledMotor(new CANTalon(7), new JoystickYAxis(auxStick));
		ropeMotor2.constrain(0.0, 1.0);
		ropeMotor2.getMotor().setInverted(true);*/
		
		ropeMotor1=new FRCMotor(new CANTalon(6));
		ropeMotor2=new FRCMotor(new CANTalon(7));
		ropeMotor1.getMotor().setInverted(true);
		ropeMotor2.getMotor().setInverted(true);
		setClimbSpeed(0);
				
		FRCButton climbFast=new FRCButton(auxStick,ClimbFastID);
		FRCButton climbSlow=new FRCButton(auxStick,ClimbSlowID);
		
		climbFast.setAction(new Action(){

			@Override
			public void start() {

			}

			@Override
			public void enabled() {
				setClimbSpeed(1);
			}

			@Override
			public void stop() {
				setClimbSpeed(0);
			}
			@Override
			public void disabled() {

			}
		});

		climbSlow.setAction(new Action(){

			@Override
			public void start() {

			}

			@Override
				public void enabled() {
					setClimbSpeed(0.5);
				}

			@Override
			public void stop() {
				setClimbSpeed(0);
			}

			@Override
			public void disabled() {

			}
		});
		
		//Shooter motors
		ballSpinny=new FRCMotor(new CANTalon(9));
		ballShooty=new FRCMotor(new CANTalon(10));
		
		//Shooter Triggers
		FRCButton shootTrigger=new FRCButton(auxStick,0);
		FRCButton agitatorOverride=new FRCButton(auxStick,4);
		
		shootTrigger.setAction(new Action(){

			@Override
			public void start() {
				shootStartTime=Updater.getTime();
			}

			@Override
			public void enabled() {
				ballShooty.set(getShootSpeed());
				if(Updater.getTime()-shootStartTime>1)
				{
					ballSpinny.set(SPINNY_POWER);
				}
			}

			@Override
			public void stop() {
				ballSpinny.set(0);
			}

			@Override
			public void disabled() {
				ballShooty.set(0);
			}
		});

		
		agitatorOverride.setAction(new Action(){

			@Override
			public void start() {
				ballSpinny.set(SPINNY_POWER);
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {
				ballSpinny.set(0);
			}

			@Override
			public void disabled() {

			}
		});

		//input.setSensitivity(0.5, 0.3);
		
		Deadzone deadzone=new Deadzone();
		
		accelLimit=new AccelLimit(4,16);
		
		/*
		//Full speed button
		Button fullSpeed = new JoystickButton(driveStick, FullSpeedButtonID);
		fullSpeed.setPressAction(new ButtonAction() {
			@Override
			public void onAction() {
				input.setSensitivity(1.0, 0.5);
			}
		});
		fullSpeed.setReleaseAction(new ButtonAction() {
			@Override
			public void onAction() {
				input.setSensitivity(0.5, 0.3);
			}
		});*/
		
		//Gyro lock
		navX = new NavXRollInput(Port.kMXP);
		PID gyroPID = new PID(0.18*13.75,0,.0003*13.75);
		gyroPID.setInput(navX);
		GyroCorrection gCorrect=new GyroCorrection(navX,gyroPID,20,0.3*20);
		GyroLock gLock = new GyroLock(gCorrect);
		
		new FRCButton(driveStick, LeftButtonID).setAction(new Action() {
			@Override
			public void start() {
				gLock.setTargetAngle(gLock.getTargetAngle().add(new Degrees(-5)));
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		});
		new FRCButton(driveStick, RightButtonID).setAction(new Action() {
			@Override
			public void start() {
				gLock.setTargetAngle(gLock.getTargetAngle().add(new Degrees(5)));
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		});
		
		//DriveTrain joystick input
		ArcadeDriveInput input = new ArcadeDriveInput(driveStick);
		Sensitivity sensitivity = new Sensitivity(1, SLOW_SPIN_SPEED);

		//Power curve
		PowerCurve curve = new PowerCurve(2);
		
		
		//Gyro lock button
		new FRCButton(driveStick, GyroLockButtonID).;

		
		//FIELD CENTRIC DRIVE!!!!
		FieldCentricDriveInput fieldCentric=new FieldCentricDriveInput(driveStick,gCorrect);
		new FRCButton(driveStick,FieldCentricButtonID);
		//END FIELD CENTRIC. :(
		
		
		FRCButton resetGyroButton=new FRCButton(driveStick,ResetGyroID);
		resetGyroButton.setAction(new Action(){

			@Override
			public void start() {
				gCorrect.reset();
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		});
		
		
		//Vision
		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    //camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		/*Vision vision=new Vision(camera);
		//VisionStep visionStep=new VisionStep(IMG_WIDTH/2, vision, -0.0001, 0, -0.00001, 10);
		
		//new ToggleButton(driveStick, VisionButtonID, visionStep);*/
		
		CameraServer server=CameraServer.getInstance();
		server.startAutomaticCapture();
		//server.startAutomaticCapture();
		
		/*CameraServers server=new CameraServers("cam0","cam1");
		server.start();
		
		
		Button frontCam=new JoystickButton(auxStick,11);
		Button rearCam=new JoystickButton(auxStick,10);
		
		frontCam.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				server.switchTo(0);
			}});
		
		rearCam.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				server.switchTo(1);
			}});
		*/
		//DriveTrain wheels
		builder = new DriveTrainBuilder();
		builder.setDriveTrainType(DriveTrainType.TANK);
		
		//PID motorPID = new PID(0.5, 0.05, 0);
		PID motorPID = new PID(0, 0, 0);
		encRight = new SEncoder(new FRCEncoder(2, 3), /*5865/76.25/*952.0/(6.0*Math.PI)*/18208/239.4);
		encLeft = new SEncoder(new FRCEncoder(4, 5), /*5865/76.25/*952.0/(6.0*Math.PI)*/18208/239.4);
		
		builder.addDriveModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO, encLeft, motorPID.copy(), MotorInputType.SPEED, new FRCMotor(new CANTalon(3)), new FRCMotor(new CANTalon(4))));
		builder.addDriveModule(new DriveModule(new XY(13.75, 0), new Degrees(180), encRight, motorPID.copy(), MotorInputType.SPEED, new FRCMotor(new CANTalon(1)), new FRCMotor(new CANTalon(2))));
		//builder.addDriveModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO, maxSpeed, new Motor(new CANTalon(3)), new Motor(new CANTalon(4))));
		//builder.addDriveModule(new DriveModule(new XY(13.75, 0), new Degrees(180), maxSpeed, new Motor(new CANTalon(1)), new Motor(new CANTalon(2))));

		

		
		
		
		builder.setInput(input);
		builder.addStep(deadzone);
		builder.addStep(sensitivity);
		builder.addStep(curve);
		builder.addStep(accelLimit);
		//builder.addStep(visionStep);
		builder.addStep(gCorrect);
		
		try {
			builder.build();
		} catch (InvalidDriveTrainException e) {
			e.printStackTrace();
		}
		
		pdp = new PowerDistributionPanel();
		
		FRCButton gyroLockOff= new FRCButton(auxStick,GyroLockOff);
		gyroLockOff.setAction(new Action() {

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}

			@Override
			public void start() {
				gLock.stop();
			}});
		
		FRCButton gearModeAuto=new FRCButton(auxStick,GearButtonAutoID);
		gearModeAuto.setAction(new Action(){

			@Override
			public void start() {
				gearMode=GEAR_MODE.AUTO;
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		});
		FRCButton gearModeManual=new FRCButton(auxStick,GearButtonManualID);
		gearModeManual.setAction(new Action(){

			@Override
			public void start() {
				gearMode=GEAR_MODE.MANUAL;
				gear.stop();
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		});
		
		//FULL SPIN BUTTON
		FRCButton fullSpinButton=new FRCButton(driveStick,FullSpinButtonID);
		fullSpinButton.setAction(new Action(){

			@Override
			public void start() {
				sensitivity.turnScale = FULL_SPIN_SPEED;
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {
				sensitivity.turnScale = SLOW_SPIN_SPEED;
			}

			@Override
			public void disabled() {

			}
		});
		
		
		//==================== AUTO SUBROUTINES ====================
		
		StateMachine dropGear=new StateMachine(
				new DriveEncoders(20, 0.3),
				//new WiggleLeft(),
				//new WiggleRight(),
				new DriveTime(0.25,0.2),
				new GearOpenState(gear),
				new DriveEncoders(-22, -0.3));

		State resetGyro= new State(){

			@Override
			public boolean isDone() {
				return true;
			}

			@Override
			public void start() {
				gCorrect.reset();
			}

			@Override
			public void enabled() {

			}

			@Override
			public void stop() {

			}

			@Override
			public void disabled() {

			}
		};
		
		
		//==================== TEST AUTO MODES ====================
		//TODO: FIX
		AutoMode autoDrive=new AutoMode("AutoDriveEncoders", new DriveEncoderGyro(new DashboardInput("drive-enc", 50), new DashboardInput("drive-enc-speed", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect));
		super.addAutoMode(autoDrive);
		
		AutoMode autoSmartDashboardTest=new AutoMode("AutoSmartDashboardTest (sdtest)",
				new DriveEncoders(new DashboardInput("sdtest-drive1-dist", 50), new DashboardInput("sdtest-drive1-power", 0.5)),
				new Delay(new DashboardInput("sdtest-delay")),
				new DriveEncoders(new DashboardInput("sdtest-drive2-dist", 50), new DashboardInput("sdtest-drive2-power", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS));
		super.addAutoMode(autoSmartDashboardTest);
		
		
		//==================== REAL AUTO MODES ====================
		double
			STRAIGHT_DRIVE_A=(110-36-22),//up to the peg
			SIDE_DRIVE_A=88-36,//first drive to the turn //52
			SIDE_DRIVE_B=(61.5-22+16),//from the turn to the peg  //55.5
			SIDE_DRIVE_C=100;//after backing up, across the baseline
		
		
		/*
		 * Left peg
		 * 88-25
		 * 52 degrees
		 * 61.5-22 forward
		 * 
		 * 
		 * Right peg
		 * 86 inch
		 * 53 degrees
		 * 4 inches
		 */
		
		double FULL_SPEED=0.2;//0.8
		
		
		super.addAutoMode(new AutoMode("Gear STRAIGHT Then Nothing Else", 
				resetGyro,
				new DriveEncoderGyro(STRAIGHT_DRIVE_A, FULL_SPEED, 0, true, gCorrect),
				dropGear));
		
		super.addAutoMode(new AutoMode("Gear LEFT Peg (Turn RIGHT)",
				resetGyro,
				new DriveEncoderGyro(SIDE_DRIVE_B,FULL_SPEED,52, true, gCorrect),
				dropGear,
				new DriveEncoderGyro(-SIDE_DRIVE_B, FULL_SPEED, 52,false, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C,FULL_SPEED, 0,false, gCorrect)));
		
		super.addAutoMode(new AutoMode("Gear RIGHT Peg (Turn LEFT)",
				resetGyro,
				new DriveEncoderGyro(SIDE_DRIVE_A, FULL_SPEED, 0, false, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_B, FULL_SPEED, -52,false, gCorrect),
				dropGear,
				new DriveEncoderGyro(-SIDE_DRIVE_B, -FULL_SPEED, -52,false, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C,FULL_SPEED, 0 ,false, gCorrect)));
		
		super.addAutoMode(new AutoMode("SPENCER",
				resetGyro,
				new DriveEncoderGyro(70, FULL_SPEED, gCorrect),
				new DriveEncoderGyro(55-22,new Degrees(52),false, FULL_SPEED, gCorrect),
				dropGear,
				new DriveEncoderGyro(-(55-22),new Degrees(52),false, -FULL_SPEED, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C,Angle.ZERO,false, FULL_SPEED, gCorrect)));
		
		super.addAutoMode(new AutoMode("Drive Forward 14 ft",
				resetGyro,
				new DriveEncoderGyro(14*12, FULL_SPEED, gCorrect)));
		
		super.addAutoMode(new AutoMode("6-90-2",
				resetGyro,
				new DriveEncoderGyro(6*12, FULL_SPEED, gCorrect),
				new DriveEncoderGyro(2*12, new Degrees (90), false, FULL_SPEED, MAX_ENC_ACCEL,MAX_ENC_TICKS,gCorrect)));
		super.addAutoMode(new AutoMode("Random Test",
				resetGyro,
				new DriveEncoderGyro(52, FULL_SPEED, gCorrect),
				new DriveEncoderGyro(72-17, new Degrees (60), false, FULL_SPEED, MAX_ENC_ACCEL,MAX_ENC_TICKS,gCorrect)));
		
		//==================== EDITABLE TEST AUTO MODES ====================
		DashboardInput
			STRAIGHT_DRIVE_A_INPUT=new DashboardInput("STRAIGHT_DRIVE_A",STRAIGHT_DRIVE_A),//up to the peg
			SIDE_DRIVE_A_INPUT=new DashboardInput("SIDE_DRIVE_A",SIDE_DRIVE_A),//first drive to the turn
			SIDE_DRIVE_B_INPUT=new DashboardInput("SIDE_DRIVE_B",SIDE_DRIVE_B),//from the turn to the peg
			SIDE_DRIVE_C_INPUT=new DashboardInput("SIDE_DRIVE_B",SIDE_DRIVE_B);//after backing up, across the baseline
		
		DashboardInput FULL_SPEED_INPUT=new DashboardInput("FULL_SPEED",FULL_SPEED);
		
		Input<Double> zeroInput=new ZeroInput();
		
		super.addAutoMode(new AutoMode("EDITABLE Gear STRAIGHT Then Nothing Else", 
				resetGyro,
				new DriveEncoderGyro(STRAIGHT_DRIVE_A_INPUT, FULL_SPEED_INPUT, gCorrect),
				dropGear));
		
		super.addAutoMode(new AutoMode("EDITABLE Gear LEFT Peg (Turn RIGHT)",
				resetGyro,
				new DriveEncoderGyro(SIDE_DRIVE_A_INPUT, FULL_SPEED_INPUT, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_B_INPUT,input60,false, FULL_SPEED_INPUT, gCorrect),
				dropGear,
				new DriveEncoderGyro(Input.neg(SIDE_DRIVE_B_INPUT),input60,false, Input.neg(FULL_SPEED_INPUT), gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C_INPUT,zeroInput,false, FULL_SPEED_INPUT, gCorrect)));
		
		super.addAutoMode(new AutoMode("EDITABLE Gear RIGHT Peg (Turn LEFT)",
				resetGyro,
				new DriveEncoderGyro(SIDE_DRIVE_A_INPUT, FULL_SPEED_INPUT, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_B_INPUT,inputNeg60,false, FULL_SPEED_INPUT, gCorrect),
				dropGear,
				new DriveEncoderGyro(Input.neg(SIDE_DRIVE_B_INPUT),inputNeg60,false, Input.neg(FULL_SPEED_INPUT), gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C_INPUT,zeroInput,false, FULL_SPEED_INPUT, gCorrect)));

		
		/*AutoMode gearLeft = new AutoMode("Gear left peg",
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", 0.35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("left-leg-2", 60), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect)
						);
		super.addAutoMode(gearLeft);
		
		AutoMode gearRight = new AutoMode("Gear right peg",
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), g-Correct, true),
				new DriveEncoderGyro(new DashboardInput("right-leg-2", -60), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect )
						);
		super.addAutoMode(gearRight);*/
		
		/*AutoMode gearLeft = new AutoMode("Gear left peg",
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", 0.35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("left-leg-2", 60), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect)
						);
		super.addAutoMode(gearLeft);
		
		AutoMode gearRight = new AutoMode("Gear right peg",
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("right-leg-2", -60), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect )
						);
		super.addAutoMode(gearRight);*/
		
		super.sendAutoModes();
		
	}

	public void update()
	{
		//Debug.msg("Close switch", closeSwitch.get() ? "true" : "false");
		//Debug.msg("Open switch", openSwitch.get() ? "true" : "false");
		
		Debug.num("encRight speed", encRight.get());
		Debug.num("encLeft speed", encLeft.get());
		Debug.num("encRight raw dist", encRight.getRawDistance());
		Debug.num("encLeft raw dist", encLeft.getRawDistance());
		Debug.num("encRight inches", encRight.getDistance().get());
		Debug.num("encLeft inches", encLeft.getDistance().get());
		Debug.num("6-amps-pdp", pdp.getCurrent(13));
		Debug.num("7-amps-pdp", pdp.getCurrent(14));
		Debug.msg("gyroAngle", navX.get());
		Debug.msg("limit-openLeft", gear.getLeftOpen());
		Debug.msg("limit-openRight", gear.getRightOpen());
		Debug.msg("limit-closeLeft", gear.getLeftClose());
		Debug.msg("limit-closeRight", gear.getRightClose());
		//SmartDashboard.putNumber("MaxTurn",SprocketRobot.getDriveTrain().getMaxTurn().toDegrees());
		boolean autoGear=auxStick.getThrottle() < 0.5;
		//if(GEAR_MODE == 1 && super.isOperatorControl() && auxStick != null && gear != null) {
		
		//}
		Debug.msg("Gear control mode", gearMode);
		
		//gear.gearSpeed = gearSpeedInput.get();
	}

	public void setClimbSpeed(double spd)
	{
		ropeMotor1.set(spd);
		ropeMotor2.set(spd);
	}
	
	public double getShootSpeed() {
		// TODO Auto-generated method stub
		return 0.75+0.25*auxStick.getY();
	}

	@Override
	public void autoInit() {
		//TODO: Badly named
		accelLimit.stop();
	}



	@Override
	public void userTeleopInit() {
		accelLimit.start();
	}
	
}

