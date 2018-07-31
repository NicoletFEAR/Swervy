package org.usfirst.frc.team4786.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

import org.usfirst.frc.team4786.robot.commands.*;
import org.usfirst.frc.team4786.robot.OI;
import org.usfirst.frc.team4786.robot.Robot;
import org.usfirst.frc.team4786.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class SwerveTrain extends Subsystem { // Like DriveTrain, but swervy

	private AHRS navX; // navX so we know angle 
	
	public boolean isFieldOriented;
	
	public static ModuleDriver backRightModule;
	public static ModuleDriver backLeftModule;
	public static ModuleDriver frontRightModule;
	public static ModuleDriver frontLeftModule;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new OpenLoopSwerveJoystick()); // sets the default command to be our open swerve driving command
    }
    
    public SwerveTrain(ModuleDriver backRight, ModuleDriver backLeft, ModuleDriver frontRight, ModuleDriver frontLeft) { // makes a swerveTarin with four modules
    	
    	navX = new AHRS(SPI.Port.kMXP); // makes the new navX
    	navX.reset(); // resets navX 
    	
    	this.isFieldOriented = (false);
    	
    	this.backRightModule = backRight;
    	this.backLeftModule = backRight;
    	this.frontRightModule = backRight;
    	this.frontLeftModule = backRight;
    	
    }
    
    public void switchControlModeOrientation () {
    	isFieldOriented = !isFieldOriented;
    }
    
    public double[] convertJoystickToIntentAxes () { // takes
    	
    	double intentAxes[] = new double[3]; // makes an array of doubles to hold the 3 joystick axes, x, y, and r
    	
    	if (Math.abs(Robot.oi.driveStick.getRawAxis(1)) <= RobotMap.driveJoystickXDeadZone) { // dead zone on the joystick so it does not move accidentaly
    		intentAxes[0] = 0;
    	} else {
    		intentAxes[0] = Robot.oi.driveStick.getRawAxis(1); // places axes 1 into index 0 of array
    	}
    	if (Math.abs(Robot.oi.driveStick.getRawAxis(2)) <= RobotMap.driveJoystickXDeadZone) { // dead zone on the joystick so it does not move accidentaly
    		intentAxes[1] = 0;
    	} else {
    		intentAxes[1] = Robot.oi.driveStick.getRawAxis(2); // places axes 2 into index 1 of array
    	}
    	if (Math.abs(Robot.oi.driveStick.getRawAxis(3)) <= RobotMap.driveJoystickXDeadZone) { // dead zone on the joystick so it does not move accidentaly
    		intentAxes[2] = 0;
    	} else {
    		intentAxes[2] = Robot.oi.driveStick.getRawAxis(3); // places axes 3 into index 2 of array
    	}
    	
    	
    	return intentAxes;
    }
    
    public double[] convertIntentToWheelAngleSpeed (double xAxisIntent, double yAxisIntake, double rAxisIntent) { // takes in x, y , and r intent and converts/outputs wheel angles and speeds
    	
    	double WheelAngleSpeeds[] = new double[8]; // makes an array of doubles to hold the target speeds and angles of the wheels
    	
    	//math :) depending on field oriented or robot oriented
    	
    	if (isFieldOriented) {
    		
    	} else { // if Robot Oriented
    		
    	}
    	
    	
    	return WheelAngleSpeeds; // outputs array: (generally called in OpenLoopSwerveJoystick)
    	// [BRRotSpd, BRDriSpd, BLRotSpd, BLDriSpd, FRRotSpd, FRDriSpd, FLRotSpd, FLDriSpd]

    }
    
    public void setModuleAngleSpeeds (double[] WheelAngleSpeedsArray) {
    	
    	backRightModule.setRotationSpeed(WheelAngleSpeedsArray[0]);
    	backRightModule.setDriveSpeed(WheelAngleSpeedsArray[1]);
    	backLeftModule.setRotationSpeed(WheelAngleSpeedsArray[2]);
    	backLeftModule.setDriveSpeed(WheelAngleSpeedsArray[3]);
    	frontRightModule.setRotationSpeed(WheelAngleSpeedsArray[4]);
    	frontRightModule.setDriveSpeed(WheelAngleSpeedsArray[5]);
    	frontLeftModule.setRotationSpeed(WheelAngleSpeedsArray[6]);
    	frontLeftModule.setDriveSpeed(WheelAngleSpeedsArray[7]);
    	
    }
    
}

