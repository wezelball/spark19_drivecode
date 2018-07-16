/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
 * spark19_drivecode
 * Jumper (Dave Cohen)
 * 
 */
package org.usfirst.frc.team384.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	/*
	 *  Sendable chooser for autonomous selection
	 *  
	 *  NOTE
	 *  Keep string length to 16 characters or less, or options may not show
	 *  (chief delphi)
	 */
	private static final String kDefaultAuto = "2 cube switch";
	private static final String k1Auto = "1 cube switch";
	private static final String k2Auto = "Right corner";
	private static final String k3Auto = "Left corner";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	// Subsystem classes
	public Claws claws;
	public OI	oi;
	public Drivetrain drivetrain;
	public Elevator elevator;
	public Pneumatics pneumatics;
	
	// Constructor
	public Robot()	{
		claws = new Claws();
		oi = new OI();
		drivetrain = new Drivetrain();
		elevator = new Elevator();
		pneumatics = new Pneumatics();
	}
	
	// Operational flags
	boolean cubePresent;		// a cube is trapped in the claws

	/*
	 * The static booleans are controlled by the methods as shown 
	 */
	static boolean isTurning = false;	// the robot is turning - controlled by drivebase.turnTo()
	static boolean isDriving = false;	// the robot is driving - controlled by drivebase.driveTo()
	static boolean isElevating = false;	// elevator is elevating - controlled by elevator.moveTo()
	
	/*
	 * Variables for control of turn, drive, and elevator controller PID's 
	 */
	double turnAngle = 0; 		// the value of the turn angle in auto
	double driveDistance = 0;	// How far to drive in auto
	int elevatorHeight = 0;		// The elevator height 
	
	/*
	 * Controls the sequencing of autonomous 
	 */
	int autoStep;						// current step number in autonomous
	boolean isSwitchLeftOurs = false;	// do we own the left scale?  if not, we own the right
	boolean isScaleLeftOurs = false;	// do we own the left scale?  if not, we own the right
	boolean sideLeft = false;			// true of on the left side of field
	boolean sideRight = false;			// true of on the left side of field
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 * 
	 * I don't think this is executed when robot is taken from disabled to enabled,
	 * unless the Rio is rebooted first.
	 * 
	 */
	@Override
	public void robotInit() {
		// Autonomous modes
		m_chooser.addDefault("2 cube switch", kDefaultAuto);
		m_chooser.addObject("1 cube switch", k1Auto);
		m_chooser.addObject("Right corner", k2Auto);
		m_chooser.addObject("Left corner", k3Auto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		// initialize the compressor 
		pneumatics.compressorSetClosedLoop(true);
		
		// Initialize encoders
		drivetrain.initializeEncoders();
		elevator.initializeEncoders();
		
		// Zero the IMU
		drivetrain.imuZeroYaw();
		
		// Set the drivetrain brakemode
		drivetrain.setBrakeMode(true);
		
		// Set the elevator brakemode
		drivetrain.setBrakeMode(true);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		
		/*
		 * Robot field arrangement from the FMS - interpret the string
		 */
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.length() > 0)
		{
		  if(gameData.charAt(0) == 'L')		
		  {
			  isSwitchLeftOurs = true;		// Our switch is on the left sode
		  } else {
			  isSwitchLeftOurs = false;		// Our switch is on the right sode
		  }

		  if(gameData.charAt(1) == 'L')		
		  {
			  isScaleLeftOurs = true;		// Our scale is on the left sode
		  } else {
			  isScaleLeftOurs = false;		// Our scale is on the right sode
		  }
		
		}

		/*
		 *  Are we going for the scale or the switch or the straight drive in auto?
		 *  
		 */
		
		elevator.initializeEncoders();	// Zero the IMU		
		autoStep = 0;	// try to comment this out
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {		
		switch (m_autoSelected) {
			case k1Auto:
				if (isSwitchLeftOurs)	{		// look at FMS data to see which side if switch is ours
					singleCubeLeftAuto();
				} else {
					singleCubeRightAuto();
				}
				break;
			case k2Auto:						// we are in right corner, pick the target
				if (!isScaleLeftOurs)	{		// the scale gets 1st priority
					singleScaleRightAuto();		// FMS says scale is on our side
				} else if (!isSwitchLeftOurs){
					singleSwitchRightAuto();	// scale is not on our side, but switch is
				} else	{
					crossAuto();				// neither is on our side, just cross the auto line
				}
				break;
			case k3Auto:		// we are in left corner, pick the target
				if (isScaleLeftOurs)	{		// the scale gets 1st priority
					singleScaleLeftAuto();		// FMS says scale is on our side
				} else if (isSwitchLeftOurs){
					singleSwitchLeftAuto();		// scale is not on our side, but switch is
				}	else	{
					crossAuto();				// neither is on our side, just cross the auto line
				}
				break;
			case kDefaultAuto:
			default:
				if (isSwitchLeftOurs)	{	// look at FMS data to see which side if switch is ours
					doubleCubeLeftAuto();
				} else {
					doubleCubeRightAuto();
				}
				break;
		}		
	}


	public void teleopInit()	{
		elevator.shiftToElevate(); 	// safer to start out ths way
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {	
		
		// STICK0 (DRIVER)
		if (oi.getButtonHeld(Constants.STICK0,1))	{
			drivetrain.arcadeDrive(oi.getAxis(Constants.STICK0, Constants.YAXIS), -oi.getAxis(Constants.STICK0, Constants.XAXIS));
		}
		
		// Wrist
		if (oi.getButtonPressed(Constants.STICK0,2))	{
			claws.toggleWrist();
		}

		// Claws
		if (oi.getButtonPressed(Constants.STICK0,3))	{
			claws.toggleClaws();
		}		
				
		
		// Slow eject the cube
		if (oi.getButtonHeld(Constants.STICK0,6))	{
			claws.intakeEject(0.5);
		}	else if (oi.getButtonReleased(Constants.STICK0, 6))	{
				claws.intakeStop();
			System.out.println("Executed intake stop");
		}
		
		// Intake auto acquire
		// Photoeye code below with take hold of cube after beam broken
		if (oi.getButtonPressed(Constants.STICK0,7))	{
				claws.intakeAcquire();
		}	else if (oi.getButtonReleased(Constants.STICK0, 7))	{
				claws.intakeStop();
		}
		
		// If the photoeye is blocked, go to low power
		// Should this be part of the Claw class?
		// out of place here, but works
		if (claws.cubePresent() && !oi.joyButtonStatus[0][6] && !oi.joyButtonStatus[0][9])	{
			claws.intakeHold();
			cubePresent = true;
			//System.out.println("joy button  stick 0, button 6" + oi.joyButtonStatus[0][6]);
		} else if (cubePresent) {
			claws.intakeStop();
			cubePresent = false;
		}
				
		// Fast eject the cube
		if (oi.getButtonHeld(Constants.STICK0,9))	{
			claws.intakeEject(1.0);
		}	else if (oi.getButtonReleased(Constants.STICK0, 9))	{
				claws.intakeStop();
				//System.out.println("Executed intake stop");
		}
		
		
		// STICK1 (COPILOT)
		
		// Manual elevator control
		if (oi.getButtonHeld(Constants.STICK1,4))	{
			elevator.manualControl(-oi.getAxis(Constants.STICK1, Constants.YAXIS));
		} else {
			elevator.stop();
		}
		
		// Toggle climber cylinder
		if (oi.getButtonPressed(Constants.STICK1, 5))	{
			elevator.climberToggle();
		}
		
		// Set PTO to elevator
		if (oi.getButtonHeld(Constants.STICK1, 6))	{
			elevator.shiftToElevate();
		}
		
		// Set PTO to climber
		if (oi.getButtonHeld(Constants.STICK1, 7))	{
			elevator.shiftToClimb();
		}
					
		// Send elevator to switch height position 
		if (oi.getButtonHeld(Constants.STICK1,8))	{
			elevatorHeight = Constants.kEncoder_LowSwitch;
			elevator.moveTo(elevatorHeight);
		}
		
		// Send elevator to scale height position 
		if (oi.getButtonHeld(Constants.STICK1,9))	{
			elevatorHeight = Constants.kEncoder_Scale;
			elevator.moveTo(elevatorHeight);
		}
		
		// Send elevator to ground height position 
		if (oi.getButtonHeld(Constants.STICK1,10))	{
			elevatorHeight = 0;
			elevator.moveTo(elevatorHeight);
		}
		
		if (isElevating)	{
			elevator.moveTo(elevatorHeight);
		}
		
		// SmartDashboard
		//SmartDashboard.putNumber("Joystick X axis: ", oi.getAxis(Constants.STICK0, Constants.XAXIS));
		//SmartDashboard.putNumber("Joystick y axis: ", oi.getAxis(Constants.STICK0, Constants.YAXIS));
		//SmartDashboard.putNumber("Joystick z axis: ", oi.getAxis(Constants.STICK0, Constants.ZAXIS));
		//SmartDashboard.putNumber("Left encoder position: ", drivetrain.getEncoderPosition(Constants.DRIVE_LEFT));
		//SmartDashboard.putNumber("Right encoder position: ", drivetrain.getEncoderPosition(Constants.DRIVE_RIGHT));
		SmartDashboard.putNumber("Elevator position from getEncoderPosition: ", elevator.getEncoderPosition());
		//SmartDashboard.putNumber("Elevator position from getPosition: ", elevator.getPosition());
		//SmartDashboard.putBoolean("Cube photoeye: ", claws.cubePresent());
		//SmartDashboard.putBoolean("Upper elevator prox: ", elevator.getIsElevAtTop());
		//SmartDashboard.putBoolean("Lower elevator prox: ", elevator.getIsElevAtBottom());
		// Need to scale velocities to real units
		//SmartDashboard.putNumber("Left side velocity: ", drivetrain.getEncoderVelocity(DRIVE_LEFT));
		//SmartDashboard.putNumber("Right side velocity: ", drivetrain.getEncoderVelocity(DRIVE_RIGHT));
		
		//SmartDashboard.putNumber("NavX heading: ", drivetrain.imuGetYaw());
		//SmartDashboard.putNumber("Angle: ", turnAngle);
		//SmartDashboard.putNumber("Left drive PID: ", drivetrain.leftStickValue);
		//SmartDashboard.putNumber("Right drive PID: ", drivetrain.rightStickValue);
		//SmartDashboard.putBoolean("Robot is turning: ", isTurning);
		//SmartDashboard.putBoolean("Robot is driving: ", isDriving);
		//SmartDashboard.putNumber("Current distance error: ", drivetrain.currentDistanceError);
		//SmartDashboard.putNumber("Left motor PID output ", drivetrain.getDriveMotorOutput(0));
		//SmartDashboard.putNumber("Right motor PID output ", drivetrain.getDriveMotorOutput(1));
		
	}

	/*
	 *  Override the testInit() method
	 */
	@Override
	public void testInit()	{
		//LiveWindow.addActuator("Left drive", drivetrain.frontLeftMotor);
		drivetrain.imuZeroYaw();
	}
	
	double timeout = 0;	// varies based on distance/angle
	
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

		/*
		 * TEST CODE FOR NAV TURN AND DRIVE - START
		 */ 
		
		// Zero out the Yaw axis, and zero the encoders
		if (oi.getButtonHeld(Constants.STICK1,11))	{
			drivetrain.imuZeroYaw();
			drivetrain.initializeEncoders();
		}  
						
		// Turn 90 degrees CCW
		if (oi.getButtonHeld(Constants.STICK1,8))	{
			turnAngle = -45;
			drivetrain.turnTo(turnAngle, 2.0);
		}
						
		// Turn 90 degrees CW
		if (oi.getButtonHeld(Constants.STICK1,9))	{
			turnAngle = 45;
			drivetrain.turnTo(turnAngle, 2.0);
		}
						
		// Turn to 0 degrees
		if (oi.getButtonHeld(Constants.STICK1,10))	{
			turnAngle = 0;
			drivetrain.turnTo(turnAngle, 2.0);
		}		
				
		if (isTurning)	{
			drivetrain.turnTo(turnAngle, 2.0);
		}
				
		// Go forward in inches
		if (oi.getButtonHeld(Constants.STICK1,6))	{
			driveDistance = 24.0;		// 48.0, 88.0, 240.0
			timeout = 3.0;				// adjust base on drive distance
			drivetrain.driveTo(driveDistance,timeout);
		}
		
		// Go forward in inches
		if (oi.getButtonHeld(Constants.STICK1,7))	{
			driveDistance = 90.0;		// 48.0, 88.0, 240.0
			timeout = 10.0;				// adjust base on drive distance
			drivetrain.driveTo(driveDistance, 10.0);
		}

		if (isDriving)	{
			drivetrain.driveTo(driveDistance, timeout);
		}
						
		/*
		* TEST CODE FOR NAV TURN AND DRIVE - END
		*/
	}

	public void singleCubeRightAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward
			do {
				result = drivetrain.driveTo(37.0, 1.5);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			} 
		break;
		case 1:		// turn cw, raise elevator to switch height
			do {
				result = drivetrain.turnTo(25.0, 2.0);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
								
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			}
			break;
		case 2:		// drive forward				
			do {
				result = drivetrain.driveTo(72.0, 1.5);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;	
			}
			break;
		case 3:	// lower claw, fire cube
			claws.lowerWrist();
			Timer.delay(0.25);
			claws.intakeEject(0.5);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;	// we are done with autonomous
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}

	public void singleCubeLeftAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward
			do {
				result = drivetrain.driveTo(34.0, 1.5);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			} 
		break;
		case 1:		// turn ccw, raise elevator to switch height
			do {
				result = drivetrain.turnTo(-30.0, 2.0);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
								
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			}
			break;
		case 2:		// drive forward				
			do {
				result = drivetrain.driveTo(83.0, 1.5);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;	
			}
			break;
		case 3:	// lower claw, fire cube
			claws.lowerWrist();
			Timer.delay(0.25);
			claws.intakeEject(0.5);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;	// we are done with autonomous
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}
	
	public void doubleCubeRightAuto()	{
		int result;
				
		switch (autoStep)	{
		case 0:	// drive forward
			do {
				result = drivetrain.driveTo(37.0, 1.75);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			} 
		break;
		case 1:		// turn cw, raise elevator to switch height
			do {
				result = drivetrain.turnTo(27.0, 1.0);		// 25.0
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
								
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			}
			break;
		case 2:		// drive forward				
			do {
				result = drivetrain.driveTo(77.0, 1.5);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;	
			}
			break;
		case 3:	// lower claw, fire cube
			claws.lowerWrist();
			Timer.delay(0.25);
			claws.intakeEject(0.5);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep +=1;	// we are done with autonomous
			break;
		case 4:		// back up, lower elevator, open jaws, lower wrist
			do {
				result = drivetrain.driveTo(-93.0, 4.0);
				elevator.moveTo(0);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
				claws.open();
				claws.lowerWrist();
			}
		break;
		case 5:		// turn ccw facing pile, engage intake
			do {
				result = drivetrain.turnTo(-35.0, 1.25);
			} while (result == 1);
								
			claws.intakeAcquire();	// turn on the jaws
			
			if (result == 0 || result == -1) {
				autoStep +=1;
				//autoStep = 99;
			}
		case 6:		// drive forward, stop, close jaws
			do {
				result = drivetrain.driveTo(22.0, 1.0);
				elevator.goToBottom();		// force the elevator down
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				//claws.intakeAcquire();
				claws.toggleClaws();
				claws.intakeAcquire();
				Timer.delay(0.5);
				claws.raiseWrist();
			}
			autoStep += 1;
		case 7:		// turn cw to face switch
			do {
				claws.intakeHold();
				result = drivetrain.turnTo(55.0, 2.0);	// 45.0
			} while (result == 1);
			
			autoStep += 1;
		case 8:		// drive to switch, raise elevator
			do {
				claws.intakeHold();
				result = drivetrain.driveTo(52.0, 1.5);
				elevator.moveTo(1200);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
		case 9:		// turn ccw towards switch
			do {
				result = drivetrain.turnTo(-45.0, 1.0);
				elevator.moveTo(1200);
			} while (result == 1);
	
			autoStep += 1;
			break;
		case 10:	// final short driveTo to break plane and fire
			do {
				claws.lowerWrist();
				claws.intakeEject(0.25);
				elevator.moveTo(1200);
				result = drivetrain.driveTo(12.0, 1.0);
			} while(result == 1);
			
			Timer.delay(0.50);
			claws.intakeStop();
			autoStep = 99;
			
			if (result == 0 || result == -1) {
				autoStep += 1; 
			}
			
			elevator.stop();
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}
	
	public void doubleCubeLeftAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward
			do {
				result = drivetrain.driveTo(34.0, 1.5);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			} 
		break;
		case 1:		// turn ccw, raise elevator to switch height
			do {
				result = drivetrain.turnTo(-35.0, 1.0);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
								
			if (result == 0 || result == -1) {
				autoStep +=1;	// success, advance to step 2
			}
			break;
		case 2:		// drive forward				
			do {
				result = drivetrain.driveTo(90.0, 1.5);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;	
			}
			break;
		case 3:	// lower claw, fire cube
			claws.lowerWrist();
			Timer.delay(0.25);
			claws.intakeEject(0.5);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep += 1;	// we are done with autonomous
			break;
		case 4:		// back up, lower elevator, open jaws, lower wrist
			do {
				result = drivetrain.driveTo(-95.0, 4.0);
				elevator.moveTo(0);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
				claws.open();
				claws.lowerWrist();
			}
		break;
		case 5:		// turn cw facing pile, engage intake
			do {
				result = drivetrain.turnTo(35.0, 1.5);	// 28.5
			} while (result == 1);
								
			claws.intakeAcquire();	// turn on the jaws
			
			if (result == 0 || result == -1) {
				autoStep +=1;
				//autoStep = 99;
			}
		case 6:		// drive forward to pile, stop, close jaws
			do {
				result = drivetrain.driveTo(22.0, 1.0);
				elevator.goToBottom();		// force the elevator down
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				//claws.intakeAcquire();
				claws.toggleClaws();
				claws.intakeAcquire();
				Timer.delay(0.5);
				claws.raiseWrist();
			}
			autoStep += 1;
		case 7:		// turn cw to face switch
			do {
				claws.intakeHold();
				result = drivetrain.turnTo(-57.0, 2.0);
			} while (result == 1);
			
			autoStep += 1;
		case 8:		// drive to switch, raise elevator
			do {
				claws.intakeHold();
				result = drivetrain.driveTo(50.0, 2.0);
				elevator.moveTo(1200);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
		case 9:		// turn ccw towards switch and fire
			do {
				result = drivetrain.turnTo(45.0, 1.0);
				elevator.moveTo(1200);
			} while (result == 1);
			
			autoStep += 1;
		
		case 10:
			do	{
				result = elevator.moveTo(1200);
			} while(result == 1);
			
			if (result == 0 || result == -1) {
				autoStep += 1; 
			}
			
		case 11:	// final short driveTo to break plane and fire
			do {
				result = drivetrain.driveTo(8.0, 1.0);
				claws.lowerWrist();
				claws.intakeEject(0.3);
				//elevator.moveTo(1200);
			} while(result == 1);	
			
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;
			
			if (result == 0 || result == -1) {
				autoStep += 1; 
			}
			break;
		
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}


	// Put right auto here
	public void singleScaleRightAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward, stop
			do {
				result = drivetrain.driveTo(305.0, 5.0);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep += 1;
			} 
			break;
		case 1:		// raise elevator
			do {
				result = elevator.moveTo(Constants.kEncoder_Scale);
			} while (result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			break;			
		case 2:		// turn ccw				
			do {
				result = drivetrain.turnTo(-90, 3.0);
				elevator.moveTo(Constants.kEncoder_Scale);
			} while (result == 1);
			
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			break;
		case 3:	// lower claw, fire cube
			do {
				result = drivetrain.driveTo(-12, 1.0);
				elevator.moveTo(Constants.kEncoder_Scale);
			} while (result == 1);
			
			elevator.stop();
			claws.lowerWrist();
			//Timer.delay(0.25);
			claws.intakeEject(0.8);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;	// we are done with autonomous
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}
	
	// Put right auto here
	public void singleScaleLeftAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward, stop
			do {
				result = drivetrain.driveTo(305.0, 5.0);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep += 1;
			} 
		break;
		case 1:		// raise elevator
			do {
				result = elevator.moveTo(Constants.kEncoder_Scale);
			} while (result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			break;			
		case 2:		// turn cw				
			do {
				result = drivetrain.turnTo(90, 3.0);
				//elevator.stop();
				elevator.moveTo(Constants.kEncoder_Scale);
			} while (result == 1);
								
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			break;
		case 3:	// lower claw, fire cube
			do {
				result = drivetrain.driveTo(-12, 1.0);
				elevator.moveTo(Constants.kEncoder_Scale);
				} while (result == 1);
			
			elevator.stop();
			claws.lowerWrist();
			//Timer.delay(0.25);
			claws.intakeEject(0.8);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;	// we are done with autonomous
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}
	
	public void singleSwitchRightAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward, stop
			do {
				result = drivetrain.driveTo(150.0, 3.0);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep += 1;
			} 
		break;
		case 1:		// raise elevator
			do {
				result = elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			break;			
		case 2:		// turn ccw				
			do {
				result = drivetrain.turnTo(-90, 2.0);
				//elevator.stop();
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
								
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			
		case 3:		// drive forward to switch
			do {
				result = drivetrain.driveTo(19.0, 2.0);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep += 1;
			}
			
			break;
		case 4:	// lower claw, fire cube
			elevator.moveTo(Constants.kEncoder_LowSwitch);
			claws.lowerWrist();
			Timer.delay(0.25);
			claws.intakeEject(0.8);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;	// we are done with autonomous
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}
		
	public void singleSwitchLeftAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward, stop
			do {
				result = drivetrain.driveTo(150.0, 3.0);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep += 1;
			} 
		break;
		case 1:		// raise elevator
			do {
				result = elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			break;			
		case 2:		// turn ccw				
			do {
				result = drivetrain.turnTo(90, 2.0);
				//elevator.stop();
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);
								
			
			if (result == 0 || result == -1) {
				autoStep +=1;
			}
			
		case 3:		// drive forward to switch
			do {
				result = drivetrain.driveTo(19.0, 2.0);
				elevator.moveTo(Constants.kEncoder_LowSwitch);
			} while (result == 1);		
			
			if (result == 0 || result == -1) {
				autoStep += 1;
			}
			
			break;
		case 4:	// lower claw, fire cube
			elevator.moveTo(Constants.kEncoder_LowSwitch);
			claws.lowerWrist();
			Timer.delay(0.25);
			claws.intakeEject(0.8);
			Timer.delay(0.5);
			claws.intakeStop();
			autoStep = 99;	// we are done with autonomous
			break;
		case 99:	// autonomous finished (or failed)
			break;
			
		default:
			System.out.println("Default reached, illegal value");
		}
	}

	// Just drive past the auto line
	public void crossAuto()	{
		int result;
		
		switch (autoStep)	{
		case 0:	// drive forward
			do {
				result = drivetrain.driveTo(135.0, 12.0);	// reduce this distance
			} while (result == 1);		
				
			if (result == 0 || result == -1) {
				autoStep = 99;
			}
		case 99:	// autonomous finished (or failed)
			break;
		default:
			System.out.println("Default reached, illegal value");
			break;
		}
	}
}
