package org.usfirst.frc.team4786.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;

public class RobotMap {
	
	public static double wheelAnglePValue = 1;
	
	public static double wheelWidthDistance = 20; // distance between wheels left to right
	public static double wheelLengthDistance = 18.5; // distance between wheels front to back
	
	public static double encoderCodesPerRev = 1024; // how many ticks are in an encoder circle (may be wrong)
	
	// Joystick dead zones
	public static double driveJoystickXDeadZone = 0.05;
	public static double driveJoystickYDeadZone = 0.05;
	public static double driveJoystickzDeadZone = 0.075;
	
	// Our drive motor Talon ID's:
	public static int backRightAngleMotorID = 31; // back right
	public static int backRightDriveMotorID = 30;
	
	public static int backLeftAngleMotorID = 41; // back left
	public static int backLeftDriveMotorID = 40;
	
	public static int frontRightAngleMotorID = 11; // front right
	public static int frontRightDriveMotorID = 10;
	
	public static int frontLeftAngleMotorID = 21; // front left
	public static int frontLeftDriveMotorID = 20;
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	public static void init() {
		
	}
}
