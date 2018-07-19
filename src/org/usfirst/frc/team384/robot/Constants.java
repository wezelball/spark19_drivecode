package org.usfirst.frc.team384.robot;

public class Constants {
	// Joysticks
	public static final int STICK0 = 0;
	public static final int STICK1 = 1;
	public static final int XAXIS	= 0;
	public static final int YAXIS	= 1;
	public static final int ZAXIS	= 2;
	public static final int ZROTATE	= 3;	// only for Saitek, get rid of for student sticks
	
	// Drivetrain
	public static final int DRIVE_LEFT = 0;
	public static final int DRIVE_RIGHT = 1;
	public static final int DRIVE_ENC_PPR  = 512;	// Talon counts 4 edges * 128 PPR
	// Reducing wheel diameter make robot go farther for given drive distance
	public static final double DRIVE_DIST_PER_PULSE = 5.875 * Math.PI / DRIVE_ENC_PPR;	// units are in inches
	
	// Drivetrain turning PID
	public static final double kTurn_P = 0.15;		// 0.03
	public static final double kTurn_Pa = 0.100;		
	public static final double kTurn_I = 0.0;	// 0.0002
	public static final double kTurn_Ia = 0.0002;
	public static final double kTurn_D = 0.1;
	public static final double kTurn_F = 0.0;
    static final double kToleranceDegrees = 2.0f;	
	
    // Drivetrain driving PID
 	public static final double kDrive_P = 0.025;	// far away P gain - 0.060
 	public static final double kDrive_Pa = 0.095;	// approach P gain - 0.08
 	public static final double kDrive_I = 0.0;		// far away I gain
 	public static final double kDrive_Ia = 0.0001;	// approach I gain - 0.0005
 	public static final double kDrive_D = 0.0;
 	public static final double kDrive_F = 0.0;
 	public static final double kToleranceDistance = 0.5;
 	
 	public static final double kDriveYaw_P = 0.02;
 	public static final double kDriveYaw_I = 0.0001;
 	public static final double kDriveYaw_D = 0.0;
 	public static final double kDriveYaw_F = 0.0;
    public static final double kSteerYawToleranceDegrees = 1.0;	// units are in inches

    // Elevator  PID
  	public static final double kElevate_Pu = 0.002;	// up elevator P gain
  	public static final double kElevate_Iu = 0.0;		// up elevator I gain
  	public static final double kElevate_Du = 0.0;		// up elevator D gain
  	public static final double kElevate_Pd = 0.001;	// up elevator P gain
  	public static final double kElevate_Id = 0.0;		// up elevator I gain
  	public static final double kElevate_Dd = 0.01;		// up elevator I gain
  	
  	public static final double kElevate_Pua = 0.0015;	// up elevator P gain
  	public static final double kElevate_Iua = 0.0;	// up elevator I gain
  	//public static final double kElevate_Dua = 0.0;		// up elevator D gain
  	public static final double kElevate_Pda = 0.0002;	// up elevator P gain
  	public static final double kElevate_Ida = 0.0;	// up elevator I gain
  	//public static final double kElevate_Dda = 0.01;		// up elevator I gain
  	
  	public static final int kElevatorToleranceDistance = 300;
    
    
    /*
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/*
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;

	/* choose so that Talon does not report sensor out of phase */
	public static final boolean kLeftSensorPhase = false;
	public static final boolean kRightSensorPhase = false;

	/* choose based on what direction you want to be positive,
		this does not affect motor invert. */
	public static final boolean kLeftMotorInvert = true;
	public static final boolean kRightMotorInvert = true;
	
	// Elevator
	//public static final int ENCODER_GROUND = 0; 
	//public static final int ENCODER_LOW = 550; 
	public static final int kEncoder_LowSwitch = 950;		// 1200
	//public static final int ENCODER_HISWITCH = 1500; 		// 1100
	public static final int kEncoder_Scale = 3100;			// 3175
	public static final int kEncoder_max = 3200;			// verify this
	 
	
}
