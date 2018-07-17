package org.usfirst.frc.team384.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Claws {
	
	// Intake motors
	private WPI_TalonSRX leftWheelMotor = new WPI_TalonSRX(11);
	private WPI_TalonSRX rightWheelMotor = new WPI_TalonSRX(12);
	
	// Cube present photoeye
	DigitalInput cubePresentPE = new DigitalInput(2);
	
	private static boolean      isClawsOpen = false;
	private static boolean      isWristDown = false;
	private static Solenoid     wrist       = new Solenoid(0,1);
	private static Solenoid     claws       = new Solenoid(0,2);
	
	
	public Claws() {
		leftWheelMotor.configContinuousCurrentLimit(5, 10);
	    leftWheelMotor.configPeakCurrentLimit(0, 10);
	    leftWheelMotor.configPeakCurrentDuration(1000, 10);

	    rightWheelMotor.configContinuousCurrentLimit(5, 10);
	    rightWheelMotor.configPeakCurrentLimit(0, 10);
	    rightWheelMotor.configPeakCurrentDuration(1000, 10);
	}

	public void open() {
	  claws.set(true);
	  isClawsOpen = true;
	}

	public void close() {
	  claws.set(false);
	  isClawsOpen = false;
	}

	public boolean isClawsOpen() {
	  return isClawsOpen;
	}

	public void toggleClaws() {
	  if (isClawsOpen) {
	    close();
	  } else {
	    open();
	  }
	}

	public void lowerWrist() {
	  wrist.set(true);
	  isWristDown = true;
	}

	public void raiseWrist() {
	  wrist.set(false);
	  isWristDown = false;
	}

	public boolean isWristDown() {
	  return isWristDown;
	}

	public void toggleWrist() {
	  if (isWristDown) {
	    raiseWrist();
	  } else {
	    lowerWrist();
	  }
	}
	
	public void intakeManualControl(double throttle) {
	    leftWheelMotor.set(ControlMode.PercentOutput, -1.0 * throttle);
	    rightWheelMotor.set(ControlMode.PercentOutput, throttle);
	  }
	public void intakeAcquire() {
	    if (!cubePresent()) {
		leftWheelMotor.set(ControlMode.PercentOutput, -1.0);
	    rightWheelMotor.set(ControlMode.PercentOutput, 1.0);
	    }
	  }

	  public void intakeEject(double speed) {
		// make sure that speed is never negative here!
	    leftWheelMotor.set(ControlMode.PercentOutput, speed);
	    rightWheelMotor.set(ControlMode.PercentOutput, -speed);
	  }

	  // Hold onto a cube if we have one
	  public void intakeHold() {
	    if (cubePresent()) {
	      leftWheelMotor.set(ControlMode.PercentOutput, -0.3);
	      rightWheelMotor.set(ControlMode.PercentOutput, 0.3);
	    } else {
	      leftWheelMotor.set(ControlMode.PercentOutput, 0.0);
	      rightWheelMotor.set(ControlMode.PercentOutput, 0.0);
	    }
	  }

	  // Stop the intake
	  public void intakeStop()	{
		  leftWheelMotor.set(ControlMode.PercentOutput, 0.0);
	      rightWheelMotor.set(ControlMode.PercentOutput, 0.0);
	  }
	  
	  public boolean cubePresent() {
		  return !cubePresentPE.get();	
	  }	
}

