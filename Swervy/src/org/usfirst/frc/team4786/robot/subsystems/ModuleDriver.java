package org.usfirst.frc.team4786.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team4786.robot.Robot;
import org.usfirst.frc.team4786.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;

/**
 *
 */
public class ModuleDriver extends Subsystem {

	public WPI_TalonSRX angleMotorSpeedController;
	public WPI_TalonSRX driveMotorSpeedController;
	public SensorCollection angleEncoder;
	//public ModuleAnglePID ModuleAnglePIDController; 
	private double oldEncoderPosition;
	private double newEncPos;
	
	public boolean wheelIsFront;

	public void setWheelIsFront(boolean wheelIsFront) {
		this.wheelIsFront = wheelIsFront;
	}

	private double a;
	private double b;
	private double c;
	
	private double ticksPerHalfRot; // half an encoder rotation, will be set from robot map 

	private double differenceToTargetInDegrees;
	private double neededAngleSpeed;
	
	public ModuleDriver (int angleMotorSpeedControllerID, int driveMotorSpeedControllerID) { // makes a module with its angleMotorSpeedControllerID and driveMotorSpeedControllerID
		wheelIsFront = true;
		
		this.angleMotorSpeedController = new WPI_TalonSRX (angleMotorSpeedControllerID); // gives each new module that is created an angleMotorSpeedController
		this.driveMotorSpeedController = new WPI_TalonSRX (driveMotorSpeedControllerID);
		
		this.angleEncoder = this.angleMotorSpeedController.getSensorCollection();
		
		//this.ModuleAnglePIDController = new ModuleAnglePID (this); // makes an angle PID controller for this ModuleDriver
		this.angleMotorSpeedController.getSensorCollection().setQuadraturePosition(0, 10);
    	this.angleEncoder.setQuadraturePosition(0, 10); // sets the encoder position to 0 when the ModuleDriver is first created
    	// the second value is timeoutMS
    	this.angleEncoder.setQuadraturePosition(0, 10); // sets the encoder position to 0 when the ModuleDriver is first created
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	
	
    public void initDefaultCommand() {
    	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void spinToAngle (double targetAngle) { // spins to a target angle
    	oldEncoderPosition = angleEncoder.getQuadraturePosition(); // gets encoder position
    	// encoder is positive clockwise, but our math is positive counterclockwise, so we have to negate the encoder values
    	oldEncoderPosition = - (oldEncoderPosition);
    	ticksPerHalfRot = RobotMap.encoderTicksPerHalf;
    	// for reference:
    	// motor speed is positive counterclockwise
    	// angle is positive counterclockwise
    	// original encoder is positive clockwise, but the previous step made it positive counterclockwise
    	
    	// CONTINUOUS code to convert encoder value to under 1 rotation:
    	 a = (oldEncoderPosition / ticksPerHalfRot); // example: 1.2 means it has gone 1 half rotation and then 2/10s of a half rotation too far
    	SmartDashboard.putNumber("A", a);
    	 b = (int) a; // get just the integer portion of a
    	 
    	 c = a - b; // get just the decimal portion of a
    	 
    	if (Math.abs(a) > 1) { // if its gone past a half rotation
    		
    		if (b % 2 == 0) { // if even (has gone around full rotation and on same side)
    			newEncPos = (c * ticksPerHalfRot);
    		} else { // if odd (has gone around half rotation and therefore on the opposite side)
    			if (oldEncoderPosition >= ticksPerHalfRot) { // if it has gone too far counterclockwise
    				newEncPos = -ticksPerHalfRot + (c * ticksPerHalfRot);	
    			} else if (oldEncoderPosition <= - ticksPerHalfRot) { // gone too far clockwise
    				newEncPos = ticksPerHalfRot + (c * ticksPerHalfRot);	
    			}
    		}
    		
    	} else { // if no continuous needs to happen:
    		newEncPos = oldEncoderPosition; // leave it because its in the right range
    	}
    	
    	// FLIPS ENCODER POS IF WHEEL FACING BACKWARDS:
    	if (wheelIsFront == false) { // facing back 
    		if (newEncPos > 0) { // if it's positive:
    			newEncPos = -(ticksPerHalfRot) + newEncPos; // rotates it 180
    		} else { // if it's negative:
    			newEncPos = (ticksPerHalfRot) + newEncPos;  // rotates it 180
    		}
    	} // END FLIP
    	
    	targetAngle = (targetAngle) * (RobotMap.encoderCodesPerRev / 360); // converts to encoder ticks
    	differenceToTargetInDegrees = (targetAngle - newEncPos) / (RobotMap.encoderCodesPerRev / 360); // calculates distance to targetAngle from current enc pos and converts to degrees
    	
    	// calculate most efficient direction and distance to get to target angle
    	if (Math.abs(differenceToTargetInDegrees) > 180) { // if it is more efficient to go around the other way :
    		if (differenceToTargetInDegrees < 0){ // if the long way would be negative (clockwise) :
    			differenceToTargetInDegrees = 360 + differenceToTargetInDegrees; // counterclockwise (+)
    		} else { // if the long way would be positive (counterclockwise) :
    			differenceToTargetInDegrees = differenceToTargetInDegrees - 360; // clockwise (-)
    		} 
    	} // if the normal way is OK then differneceToTargetInDegrees is fine as it is
    	
    	
    	
    	// FLIPPING WHEELS:
    	if (Math.abs(differenceToTargetInDegrees) > 100) { // ideally is 90 CHAAANGE!!!!!!!!!!!!!
    		
    		
    		wheelIsFront = !(wheelIsFront); // tell it that you want to flip that wheel!
    		
    		
    		
    		// ENCODER FLIPPING: (same as before)
    		if (newEncPos > 0) { // if it's positive:
    			newEncPos = -(ticksPerHalfRot) + newEncPos; // rotates it 180
    		} else { // if it's negative:
    			newEncPos = (ticksPerHalfRot) + newEncPos;  // rotates it 180
    		}
    		
    		// RECALCULATE THE DISTANCE TO TARGET ANGLE WITH NEW ENCODER VALUE: (repeat, same as above)
    		targetAngle = (targetAngle) * (RobotMap.encoderCodesPerRev / 360); // converts to encoder ticks
        	differenceToTargetInDegrees = (targetAngle - newEncPos) / (RobotMap.encoderCodesPerRev / 360); // calculates distance to targetAngle from current enc pos and converts to degrees
    		// calculate most efficient direction and distance to get to target angle
        	if (Math.abs(differenceToTargetInDegrees) > 180) { // if it is more efficient to go around the other way :
        		if (differenceToTargetInDegrees < 0){ // if the long way would be negative (clockwise) :
        			differenceToTargetInDegrees = 360 + differenceToTargetInDegrees; // counterclockwise (+)
        		} else { // if the long way would be positive (counterclockwise) :
        			differenceToTargetInDegrees = differenceToTargetInDegrees - 360; // clockwise (-)
        		} 
        	} // if the normal way is OK then differneceToTargetInDegrees is fine as it is
    	}  // END OF FLIPPING CODE
    	
    	

    	// CALCULATE NEEDED SPEEDS:
    	// set motor speeds based on DifToTarInDeg:
    	if (differenceToTargetInDegrees > 0) { // if it needs to go counterclockwise (+) :
    		if (differenceToTargetInDegrees > 90) {
    			neededAngleSpeed = 1; // 90+
    		} else if (differenceToTargetInDegrees > 60) {
    			neededAngleSpeed = 1; // 60-90
    		} else if (differenceToTargetInDegrees > 30) {
    			neededAngleSpeed = 0.5; // 30-60
    		} else if (differenceToTargetInDegrees > RobotMap.angleEncoderDeadZone) {
    			neededAngleSpeed = 0.1; // deadZone-30
    		} else {
    			neededAngleSpeed = 0; // < deadZone
    		}
    	} else {  // if it needs to go clockwise (-) :
    		if (differenceToTargetInDegrees < -90) {
    			neededAngleSpeed = -1;
    		} else if (differenceToTargetInDegrees < -60) {
    			neededAngleSpeed = -1;
    		} else if (differenceToTargetInDegrees < -30) {
    			neededAngleSpeed = -0.5;
    		} else if (differenceToTargetInDegrees < -RobotMap.angleEncoderDeadZone) {
    			neededAngleSpeed = -0.1;
    		} else {
    			neededAngleSpeed = 0;
    		}
    	}
    	
    	// if no movement has to happen, keep the wheels where they are
    	//if ((Math.abs(Robot.oi.driveStick.getRawAxis(0)) < RobotMap.driveJoystickXDeadZone) && (Math.abs(Robot.oi.driveStick.getRawAxis(1)) < RobotMap.driveJoystickYDeadZone) && (Math.abs(Robot.oi.driveStick.getRawAxis(3)) < RobotMap.driveJoystickzDeadZone)) {
    	if ((Math.abs(Robot.oi.driveStick.getRawAxis(0)) < 0.175) && (Math.abs(Robot.oi.driveStick.getRawAxis(1)) < 0.175) && (Math.abs(Robot.oi.driveStick.getRawAxis(2)) < 0.175)) {	
    		neededAngleSpeed = 0;
    		SmartDashboard.putBoolean("allInDeadZone", true);
    	} else {
    		SmartDashboard.putBoolean("allInDeadZone", false);
    		//neededAngleSpeed = 0;
    	}
    	// END OF NEEDED SPEED CALC
    	
    	angleMotorSpeedController.set(neededAngleSpeed); // sets the motor to the desired speed
    	
    }
    
    
    
    
    public void setDriveSpeed (double targetSpeed) { // receives target speed and gets the module to that speed
    	if (wheelIsFront) { // if the wheel is driving forward:
    		driveMotorSpeedController.set(targetSpeed); // sets the speed of the driveMotor to target speed
    	} else {
    		driveMotorSpeedController.set(-targetSpeed); // sets it to drive backwards if wheel is flipped
    	}
    }
    

	public double getDifferenceToTargetInDegrees() { // allows value to be accessed elsewhere
		return differenceToTargetInDegrees;
	}
	
	public double getNeededAngleSpeed() {  // allows value to be accessed elsewhere
		return neededAngleSpeed;
	}
	
	public double getA() {
		return a;
	}

	public double getB() {
		return b;
	}

	public double getC() {
		return c;
	}
	
	public double getNewEncPos() {
		return newEncPos;
	}
	
	
}

