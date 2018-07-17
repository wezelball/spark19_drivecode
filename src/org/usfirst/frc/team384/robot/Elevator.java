package org.usfirst.frc.team384.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

	// Motors
	private TalonSRX lifterSecondaryMotor = new TalonSRX(9); // Front motor
	private TalonSRX lifterPrimaryMotor = new TalonSRX(10); // Rear motor

	// Solenoids
	private Solenoid pto = new Solenoid(0, 0);
	private Solenoid climber = new Solenoid(0, 3);

	private MiniPID elevatorController; // elevator position pid controller
	double elevateToPositionRate; // elevator position pid output
	double elevatorOutput; // value to elevator motor
	double currentDistanceError = 0;

	// Need to check elapsed time for PID control
	private Timer intervalTimer = new Timer(); // PID interval timer
	private Timer failTimer = new Timer(); // PID fails if value exceeded
	boolean timing = false; // PID interval timer timing

	public Elevator() {
		lifterPrimaryMotor.set(ControlMode.PercentOutput, 0);
		lifterSecondaryMotor.follow(lifterPrimaryMotor);
		lifterPrimaryMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		lifterPrimaryMotor.setSensorPhase(false);
		lifterPrimaryMotor.setInverted(false);
		lifterPrimaryMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
				LimitSwitchNormal.NormallyOpen, 0);
		lifterPrimaryMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
				LimitSwitchNormal.NormallyOpen, 0);
		lifterPrimaryMotor.overrideLimitSwitchesEnable(true);

		lifterPrimaryMotor.configOpenloopRamp(0.3, 0);
		lifterPrimaryMotor.setNeutralMode(NeutralMode.Brake);

		/*
		 * This is the PID controller for the drive
		 */
		elevatorController = new MiniPID(Constants.kElevate_Pu, Constants.kElevate_Iu, Constants.kElevate_Du);
	}

	/*
	 * Initialize encoders
	 */
	public void initializeEncoders() {
		lifterPrimaryMotor.getSensorCollection().setQuadraturePosition(0, 0);
		lifterPrimaryMotor.setSensorPhase(true); // runs negative otherwise, due to being mirrored
	}

	/*
	 * Get encoder position Consider making this a double, getting the position in
	 * inches Or the method below could return position
	 */
	public int getEncoderPosition() {
		return lifterPrimaryMotor.getSensorCollection().getQuadraturePosition();
	}

	public double getVelocity() {
		return lifterPrimaryMotor.getSelectedSensorVelocity(0);
	}

	public boolean getIsElevAtTop() {
		return lifterPrimaryMotor.getSensorCollection().isFwdLimitSwitchClosed();
	}

	public boolean getIsElevAtBottom() {
		return lifterPrimaryMotor.getSensorCollection().isRevLimitSwitchClosed();
	}

	/*
	 * The new elevator positioning code I will start with an integer position, and
	 * work up to a double position in inches
	 * 
	 * Also requires a timeout value, which is time before we give up PID control
	 * and move to whatever next step
	 * 
	 * Returns: 1 = PID not complete 0 = PID complete -1 = error
	 */
	public int moveTo(int position) {
		boolean isDescending;
		boolean inDeadBand = false;

		if (!Robot.isElevating) {
			elevatorController.setSetpoint(position);
			elevateToPositionRate = 0;
			elevatorOutput = 0.0;
			failTimer.start(); // the PID will fail if this timer exceeded
			Robot.isElevating = true;
			shiftToElevate(); // safety third!
			elevatorController.reset();
		}

		// PID constants depend on whether we are going up or down
		isDescending = (position < getEncoderPosition());

		if (currentDistanceError < (Constants.kElevatorToleranceDistance * 3))
			inDeadBand = true;

		// When we get close to the target, dynamically adjust PI terms
		if (inDeadBand) {
			if (!isDescending) { // is elevating
				elevatorController.setI(Constants.kElevate_Iu);
				elevatorController.setP(Constants.kElevate_Pu);
				elevatorController.setOutputLimits(-0.5, 1.0);
			} else {
				elevatorController.setI(Constants.kElevate_Ida);
				elevatorController.setP(Constants.kElevate_Pda);
				elevatorController.setOutputLimits(-0.5, 1.0);
			}
		}

		/*
		 * Activate elevator PID, check for proximity switches at end of travel
		 */
		currentDistanceError = Math.abs(position - getEncoderPosition());
		elevateToPositionRate = elevatorController.getOutput(getEncoderPosition(), position);
		lifterPrimaryMotor.set(ControlMode.PercentOutput, elevateToPositionRate);

		SmartDashboard.putNumber("Elevator position from getEncoderPosition: ", getEncoderPosition());
		SmartDashboard.putNumber("PID Output: ", elevateToPositionRate);

		/*
		 * Handle elevator extreme ends of travel to avoid mechanical damage
		 */
		if (getIsElevAtBottom() == true) { // If at bottom, reset encoder counts
			initializeEncoders();
			// Don't shut off completely, we'll never move again
			lifterPrimaryMotor.set(ControlMode.PercentOutput, 0.15);
		}

		if (getIsElevAtTop() == true) { // If at bottom, reset encoder counts
			// Don't shut off completely, we'll never move again
			lifterPrimaryMotor.set(ControlMode.PercentOutput, 0.15);
		}

		// Are we within deadband?
		if ((Math.abs(position - getEncoderPosition())) < Constants.kElevatorToleranceDistance) {
			if (!timing) {
				intervalTimer.start();
				timing = true;
			}
		} else {
			intervalTimer.reset();
		}

		if (intervalTimer.hasPeriodPassed(0.1)) {
			stop();
			System.out.println("Elevator PID finished, actual counts: " + getEncoderPosition());
			Robot.isElevating = false;
			intervalTimer.reset();
			failTimer.reset();
			return 0;
		} else if (failTimer.hasPeriodPassed(5.0)) { // the PID has failed!
			stop();
			System.out.println("Elevator PID timeout, setpoint: " + position);
			System.out.println("Elevator PID timeout, actual counts: " + getEncoderPosition());
			Robot.isElevating = false;
			intervalTimer.reset();
			failTimer.reset();
			return -1;
		} else { // the PID is not complete
			return 1;
		}
	}

	/*
	 * Stop the elevator
	 */
	public void stop() {
		lifterPrimaryMotor.set(ControlMode.PercentOutput, 0);
	}

	public void manualControl(double speed) {
		lifterPrimaryMotor.set(ControlMode.PercentOutput, speed);
		System.out.println("Elevator feedback (manual): " + getEncoderPosition());
		// Zero out the encoder if bottom prox made
		if (getIsElevAtBottom()) {
			lifterPrimaryMotor.getSensorCollection().setQuadraturePosition(0, 0);
		}
	}

	/*
	 * This sends the elevator down to the bottom prox at a fixed speed use only to
	 * force the elevator at bottom for cube pickup.
	 * 
	 * Could be dangerous if used from a high position
	 * 
	 * Returns true if at bottom
	 */
	public boolean goToBottom() {
		double speed = -0.5; // how fast to drop

		shiftToElevate(); // don't run the pto!
		lifterPrimaryMotor.set(ControlMode.PercentOutput, speed);
		if (getIsElevAtBottom()) {
			stop();
			return true;
		} else {
			return false;
		}
	}

	// Raises or lowers the climber arm, based on current state
	public void climberToggle() {
		if (climber.get()) {
			climber.set(false);
		} else {
			climber.set(true);
		}
	}

	public void shiftToElevate() {
		pto.set(true);
	}

	public void shiftToClimb() {
		pto.set(false);
	}

	/*
	 * Set the brake mode for the elevator
	 */
	public void setBrakeMode(boolean brakeon) {
		if (brakeon) {
			lifterPrimaryMotor.setNeutralMode(NeutralMode.Brake);
			lifterSecondaryMotor.setNeutralMode(NeutralMode.Brake);
		} else {
			lifterPrimaryMotor.setNeutralMode(NeutralMode.Coast);
			lifterSecondaryMotor.setNeutralMode(NeutralMode.Coast);
		}
	}
}
