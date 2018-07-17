package org.usfirst.frc.team384.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// for debugging, can be removed later
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
	/* Instantiate talon motors */
	private WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(7);
	private WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(8);
	private WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(5);		// -1.0 to run forward
	private WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(6);		// -1.0 to run forward

	// Need to check elapsed time for PID control
	private Timer intervalTimer = new Timer();	// PID interval timer
	private Timer failTimer = new Timer();	// PID fails if value exceeded 
	
	// Define the motors that are slaved as a control group
	private SpeedControllerGroup leftDrivetrain = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
	private SpeedControllerGroup rightDrivetrain = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
	
	// Set up differential drive for teleop
	private DifferentialDrive diffDrive = new DifferentialDrive(leftDrivetrain, rightDrivetrain);
	
	private AHRS imu;						// the NavX board
	private MiniPID turnController;			// drivebase turning pid controller
	private MiniPID driveController;		// drive distance pid controller
	private MiniPID turnYawController;		// provides steering correction for driveTo() method
	double rotateToAngleRate;				// PID output for turn PID			
	double driveToDistanceRate;
	double leftStickValue = 0.0;			
	double rightStickValue = 0.0;
	boolean timing = false;					// true if PID interval timer is timing
 	int leftDistanceError = 0;				// drive distance error
 	int rightDistanceError = 0;
 	double currentDistanceError = 0;
	
	public Drivetrain()	{
		// Initialize the IMU
		try {
			imu = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
			
		}
		
		// Do we really need this?
		frontRightMotor.setSafetyEnabled(false);
		frontLeftMotor.setSafetyEnabled(false);
		rearRightMotor.setSafetyEnabled(false);
		rearLeftMotor.setSafetyEnabled(false);
	
		/*
		 * This is the PID controller for the turn
		 */
		turnController = new MiniPID(Constants.kTurn_P, Constants.kTurn_I, Constants.kTurn_D);
		turnController.setOutputLimits(-1.0, 1.0);
		
		/*
		 * This is the PID controller for the drive
		 */
		driveController = new MiniPID(Constants.kDrive_P, Constants.kDrive_I, Constants.kDrive_D);
		driveController.setOutputLimits(-1.0, 1.0);
		
		/*
		 * This is the PID controller for the Yaw correction to keep the robot driving straight
		 * during the driveTo() method 
		 */
		turnYawController = new MiniPID
				(Constants.kDriveYaw_P, Constants.kDriveYaw_I, Constants.kDriveYaw_D);
		
		/*
		 * Configure the Talon SRX motor controllers
		 * 
		 */
		frontLeftMotor.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRightMotor.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		/* choose to ensure sensor is positive when output is positive */
		frontLeftMotor.setSensorPhase(Constants.kLeftSensorPhase);
		frontRightMotor.setSensorPhase(Constants.kRightSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		frontLeftMotor.setInverted(Constants.kLeftMotorInvert);
		frontRightMotor.setInverted(Constants.kRightMotorInvert);
		
		/* set the peak and nominal outputs, 12V means full */
		frontLeftMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		frontRightMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		
		frontLeftMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		frontRightMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		
		frontLeftMotor.configPeakOutputForward(1, Constants.kTimeoutMs);	// debug
		frontRightMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		
		frontLeftMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);	// debug
		frontRightMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
	
		rearLeftMotor.follow(frontLeftMotor); 	// follow the master talon
		rearRightMotor.setInverted(true);		
		rearRightMotor.follow(frontRightMotor);
	}
	
	
	/*
	 * Initialize encoders - set to zero
	 */
	public void initializeEncoders()	{
		frontLeftMotor.getSensorCollection().setQuadraturePosition(0, 0);
		frontRightMotor.getSensorCollection().setQuadraturePosition(0, 0);
	}

	/*
	 * Get encoder position
	 * 0 = left side drive rail
	 * 1 = right side drive rail
	 */
	public int getEncoderPosition(int side)	{
		int result;
		
		if(side == 0)	{
			result = frontLeftMotor.getSelectedSensorPosition(0);
		} else if (side == 1)	{
			result = frontRightMotor.getSelectedSensorPosition(0);
		}	else	{	// an illegal value was sent
			result = 0;
		}
		
		return result;
	}

	/*
	 * Get the encoder position as a double, averaging both sides
	 * This returns the distance in inches.
	 * Make sure that both encoder values are of same polarity
	 */
	public double getEncoderDistance()	{
		int leftEncoderDistance;
		int rightEncoderDistance;
		
		leftEncoderDistance = -frontLeftMotor.getSelectedSensorPosition(0);	// negative when forward
		rightEncoderDistance = frontRightMotor.getSelectedSensorPosition(0);
	
		// Debug
		//SmartDashboard.putNumber("Left encoder: ", leftEncoderDistance);
		//SmartDashboard.putNumber("Right encoder: ", rightEncoderDistance);
		
		/*
		 * I'm getting half of the pulses that I should on the right encoder, as if 
		 * one of the channels is not working.  I'll use the left only for now
		 * 
		 * Also, left is negative when going forward.
		 */
		
		// Return only the left encoder until the right encoder gets fixed
		return leftEncoderDistance * Constants.DRIVE_DIST_PER_PULSE;
	}
	
	/*
	 * Get encoder velocity
	 * 0 = left side drive rail
	 * 1 = right side drive rail
	 * 
	 * Talon reports velocity is sensor units per 100ms. 
	 * Current sensor 128 PPR * 4 = 512 counts per revolution
	 */
	public int getEncoderVelocity(int side)	{
		int result;
		
		if(side == 0)	{
			result = frontLeftMotor.getSelectedSensorVelocity(0);
		} else if (side == 1)	{
			result = frontRightMotor.getSelectedSensorVelocity(0);
		}	else	{	// an illegal value was sent
			result = 0;
		}
		
		return result;
	}

	/*
	 * Return the value of the output to the Talon SRX drive base motor
	 * 0 = front left
	 * 1 = front right
	 */
	public double getDriveMotorOutput(int side)	{
		double result = 0;
		
		if(side == 0)	{
			result = frontLeftMotor.getMotorOutputPercent();
		} else if (side == 1)	{
			result = frontRightMotor.getMotorOutputPercent();
		}
		
		return result;
		
	}
	
	/*
	 * Arcade drive
	 * requires stick number and axis number
	 * 
	 */
	public void arcadeDrive(double speedaxis,  double turnaxis)	{
		diffDrive.arcadeDrive(speedaxis, turnaxis);
	}
	
	/*
	 * Yaw values are in degrees, clockwise increasing positive
	 * 
	 */
	public float imuGetYaw()	{
		return imu.getYaw();
	}
	

	// FIXME
	public void imuZeroYaw()	{
		imu.zeroYaw();		// this is not working, I get a non-zero value even after calling this method
	}

	/*
	 *  Set the brake mode for the drivetrain
	 *  See  Constants.NEUTRAL_COAST and Constants.NEUTRAL_BRAKE 
	 */
	public void setBrakeMode(boolean brakeon)	{
		if (brakeon)	{
			frontRightMotor.setNeutralMode(NeutralMode.Brake);
			frontLeftMotor.setNeutralMode(NeutralMode.Brake);
			rearRightMotor.setNeutralMode(NeutralMode.Brake);
			frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		}	else {
			frontRightMotor.setNeutralMode(NeutralMode.Coast);
			frontLeftMotor.setNeutralMode(NeutralMode.Coast);
			rearRightMotor.setNeutralMode(NeutralMode.Coast);
			frontLeftMotor.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	/*
	 *  Turns the controller by specified angle.
	 *  Positive angles are clockwise
	 *  Input angle and output ranges are defined in the Drivetrain constructor
	 *  Also requires a timeout value, which is time before we give up PID control and
	 *  move to whatever next step 
	 *  
	 *  Returns:
	 *  0 if not enabled
	 *  1 if enabled and turning
	 *  -1 if error
	 */
	int turnTo(double angle, double timeout)	{
		// This is executed in first call to method
		if (!Robot.isTurning) {
			imuZeroYaw();	// I'm gonna zero the angle here
			turnController.setSetpoint(angle);
			rotateToAngleRate = 0; 	// This value will be updated in the pidWrite() method.
			leftStickValue = rightStickValue = 0.0;
			failTimer.start();		// the PID will fail if this timer exceeded
			Robot.isTurning = true;
		}
		
		// This is the final output of the PID
		rotateToAngleRate = turnController.getOutput(imu.getYaw(), angle);
		
		if (angle >= 0.0) {
			leftStickValue = -rotateToAngleRate;
			rightStickValue = rotateToAngleRate;
		} else if (angle < 0.0)	{
			leftStickValue = -rotateToAngleRate;
			rightStickValue = rotateToAngleRate;
		}
		diffDrive.tankDrive(leftStickValue,  rightStickValue);
		
		// When we get close to the target, dynamically adjust PI terms
		if (currentDistanceError < (Constants.kToleranceDegrees * 3))	{
			turnController.setI(Constants.kTurn_Ia);
			turnController.setP(Constants.kTurn_Pa);
		}
		
		// Determine if the PID is finished
		if (Math.abs(Math.abs(angle) - Math.abs(imuGetYaw())) < Constants.kToleranceDegrees ) {
			if (!timing) {
				intervalTimer.start();
				timing = true;
			}		
		} else {
			intervalTimer.reset();
			timing = false;
		}
		
		// Check to see if PID has succeeded, or timed out and failed
		if (intervalTimer.hasPeriodPassed(0.5))	{
			System.out.println("Actual angle: " + imuGetYaw());
			System.out.println("turnTo() PID finished");
			frontLeftMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);
			Robot.isTurning = false;
			intervalTimer.reset();
			failTimer.reset();
			turnController.reset();
			return 0;
		} else if (failTimer.hasPeriodPassed(timeout)) {	// the PID has failed!
			System.out.println("turnTo() PID timeout, actual angle: " + imuGetYaw());
			frontLeftMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);
			Robot.isTurning = false;
			intervalTimer.reset();
			failTimer.reset();
			turnController.reset();
			return -1;
		}
		else	{		// the PID is not complete
			return 1;
		}
	}	
    

    /*
	 *  Drives straight by a specified distance
	 *  Positive distance is forward, negative reverse
	 *  
	 *  Also requires a timeout value, which is time before we give up PID control and
	 *  move to whatever next step
	 *  
	 *  Returns:
	 *  0 if not enabled
	 *  1 if enabled and turning
	 *  -1 if error
	 */
	int driveTo(double distance, double timeout)	
	{	
		if (!Robot.isDriving) 
		{
			driveController.setSetpoint(distance);
			turnYawController.setSetpoint(imuGetYaw());
			driveController.setOutputLimits(-0.8, 0.8);
			rotateToAngleRate = 0; 	// This value will be updated in the pidWrite() method
			driveToDistanceRate = 0;
			initializeEncoders();	// zero the encoders
			leftStickValue = rightStickValue = 0.0;
			failTimer.start();		// the PID will fail if this timer exceeded
			Robot.isDriving = true;
		} 
		
		// This is the final output of the PID
		driveToDistanceRate = driveController.getOutput(getEncoderDistance(), distance);
		rotateToAngleRate = turnYawController.getOutput(imuGetYaw()); 	// setpoint already loaded
		currentDistanceError = distance - getEncoderDistance();
		leftStickValue = -driveToDistanceRate - rotateToAngleRate;
		rightStickValue = -driveToDistanceRate + rotateToAngleRate;
		diffDrive.tankDrive(leftStickValue,  rightStickValue);
		
		// When we get close to the target, dynamically adjust PI terms
		if (currentDistanceError < (Constants.kToleranceDistance * 2))	{
			driveController.setI(Constants.kDrive_Ia);	// 0.0005
			driveController.setP(Constants.kDrive_Pa);	// 0.08
		}	

		if (currentDistanceError < Constants.kToleranceDistance ) 	{
			if (!timing) {
				intervalTimer.start();
				//System.out.println("Started PID timer in driveTo");
				timing = true;
			} 
		} else 	{					
			intervalTimer.reset();
			timing = false;
		}
		
		if (intervalTimer.hasPeriodPassed(1.0))	{					// Within deadband for interval time
			frontLeftMotor.set(ControlMode.PercentOutput, 0.0);		// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);
			System.out.println("driveTo() PID finished, final error : " + (distance - getEncoderDistance()));
			Robot.isDriving = false;
			intervalTimer.reset();
			failTimer.reset();
			driveController.reset();
			turnController.reset();
			return 0;
		} else if (failTimer.hasPeriodPassed(timeout)) 	{			// the PID has failed!
			frontLeftMotor.set(ControlMode.PercentOutput, 0.0);		// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);
			System.out.println("driveTo() PID timeout, final error: " + (distance - getEncoderDistance()));
			Robot.isDriving = false;
			intervalTimer.reset();
			failTimer.reset();
			driveController.reset();
			turnController.reset();
			return -1;
		} else	{		// the PID is not complete
			return 1;
		}
	}
}

