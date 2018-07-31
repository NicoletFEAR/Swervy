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
    	this.backLeftModule = backLeft;
    	this.frontRightModule = frontRight;
    	this.frontLeftModule = frontLeft;
    	
    	
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
    	
    	
    	return intentAxes; // the three joystick axes
    }
    
    public double[] convertIntentToWheelAngleSpeed (double xAxisIntent, double yAxisIntent, double rAxisIntent) { // takes in x, y , and r intent and converts/outputs wheel angles and speeds
    	
    	double WheelAngleSpeeds[] = new double[8]; // makes an array of doubles to hold the target speeds and angles of the wheels
    	
    	//math :) depending on field oriented or robot oriented:
    	
    	double Ox1 = xAxisIntent;
    	double Oy1 = yAxisIntent;
    	
    	double x1;
    	double y1;
    	double x2 = rAxisIntent;
    		
    	if (isFieldOriented) { // change the x and y so that forward is field oriented
    		
    		double navxAngle = navX.getAngle(); // get the value of the navX
    		
    		double h = Math.sqrt ((Ox1 * Ox1) + (Oy1 * Oy1)); 
    		double thetaAngle = (Math.atan(Oy1 / Ox1) + navxAngle); // these two lines essentially add the x and y inputs into dir and mag and then rotates it based on navX angle
    		
    		x1 = h * Math.cos(thetaAngle); // converts back to x and y axes
        	y1 = h * Math.sin(thetaAngle); 
    		
    	} else { // if Robot Oriented
    		// the values should work as they are
    		x1 = xAxisIntent;
        	y1 = yAxisIntent;
    	}
    		
    		double L = RobotMap.wheelLengthDistance;  // 24.0;
    		double W = RobotMap.wheelWidthDistance;  // 18.5;
    		
    		//public void drive (double x1, double y1, double x2) {
    			
    		//mitchell's math:
    		
    		    double r = Math.sqrt ((L * L) + (W * W));
    		    y1 *= -1;

    		    double a = x1 - x2 * (L / r);
    		    double b = x1 + x2 * (L / r);
    		    double c = y1 - x2 * (W / r);
    		    double d = y1 + x2 * (W / r);

    		    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    		    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    		    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    		    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

    		    double backRightAngle = Math.atan2 (a, d) / Math.PI;
    		    double backLeftAngle = Math.atan2 (a, c) / Math.PI;
    		    double frontRightAngle = Math.atan2 (b, d) / Math.PI;
    		    double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
    		    
    		    // outputs desired speeds and angles of modules
    		    // speeds from -1 to 1 (hopefully)
    		    // angles from -180 to 180 (hopefully)
    		    
    		    WheelAngleSpeeds[0] = backRightAngle;
    		    WheelAngleSpeeds[1] = backRightSpeed;
    		    WheelAngleSpeeds[2] = backLeftAngle;
    		    WheelAngleSpeeds[3] = backLeftSpeed;
    		    WheelAngleSpeeds[4] = frontRightAngle;
    		    WheelAngleSpeeds[5] = frontRightSpeed;
    		    WheelAngleSpeeds[6] = frontLeftAngle;
    		    WheelAngleSpeeds[7] = frontLeftSpeed;
    		    
    		    // end Mitchell's math

    		//}
    		
    	
    	
    	//test
    	return WheelAngleSpeeds; // outputs array: (generally called in OpenLoopSwerveJoystick)
    	// [BRRotationTarget, BRDriveSpeed, BLRotTrg, BLDriSpd, FRRotTrg, FRDriSpd, FLRotTrg, FLDriSpd]

    }
    
    public void setModuleAngleSpeeds (double[] WheelAngleSpeedsArray) {
    	
    	backRightModule.spinToAngle(WheelAngleSpeedsArray[0]);
    	backRightModule.setDriveSpeed(WheelAngleSpeedsArray[1]);
    	backLeftModule.spinToAngle(WheelAngleSpeedsArray[2]);
    	backLeftModule.setDriveSpeed(WheelAngleSpeedsArray[3]);
    	frontRightModule.spinToAngle(WheelAngleSpeedsArray[4]);
    	frontRightModule.setDriveSpeed(WheelAngleSpeedsArray[5]);
    	frontLeftModule.spinToAngle(WheelAngleSpeedsArray[6]);
    	frontLeftModule.setDriveSpeed(WheelAngleSpeedsArray[7]);
    	
    }
    
}

