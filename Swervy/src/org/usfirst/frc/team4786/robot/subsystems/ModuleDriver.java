package org.usfirst.frc.team4786.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
	private double encoderPosition;
	private double differenceToTargetInDegrees;
	private double neededAngleSpeed;
	
	public ModuleDriver (int angleMotorSpeedControllerID, int driveMotorSpeedControllerID) { // makes a module with its angleMotorSpeedControllerID and driveMotorSpeedControllerID
		
		this.angleMotorSpeedController = new WPI_TalonSRX (angleMotorSpeedControllerID); // gives each new module that is created an angleMotorSpeedController
		this.driveMotorSpeedController = new WPI_TalonSRX (driveMotorSpeedControllerID);
		
		this.angleEncoder = this.angleMotorSpeedController.getSensorCollection();
		
		//this.ModuleAnglePIDController = new ModuleAnglePID (this); // makes an angle PID controller for this ModuleDriver

    	this.angleEncoder.setQuadraturePosition(0, 0); // sets the encoder position to 0 when the ModuleDriver is first created
    	// the second value is timeoutMS... not sure
    	
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	
	
    public void initDefaultCommand() {
    	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void spinToAngle (double targetAngle) { // spins to a target angle
    	encoderPosition = angleEncoder.getQuadraturePosition(); // gets encoder position
    	// encoder is positive clockwise, but our math is positive counterclockwise, so we have to negate the encoder values
    	encoderPosition = - (encoderPosition);
    	
    	targetAngle = (targetAngle - 90) * (RobotMap.encoderCodesPerRev / 360); // converts to forward orientation in encoder ticks
    	
    	differenceToTargetInDegrees = (targetAngle - encoderPosition) / (RobotMap.encoderCodesPerRev / 360);
    	
    	if (Math.abs(differenceToTargetInDegrees) > RobotMap.angleEncoderDeadZone) { // if it needs to move (difference greater than dead zone):
    		
    		if (differenceToTargetInDegrees > 0) { //if it needs to turn counterclockwise
    			 
    			// neededAngleSpeed =  - (Math.abs(-0.00001 * (differenceToTargetInDegrees) *(differenceToTargetInDegrees)+ 0.007 * (differenceToTargetInDegrees)- 0.06));
    			neededAngleSpeed = 1;
    			
    			
    		} else { // if it needs to turn clockwise
    			
    			//neededAngleSpeed = (Math.abs(-0.00001 * (differenceToTargetInDegrees) *(differenceToTargetInDegrees)+ 0.007 * (differenceToTargetInDegrees)- 0.06));
    			neededAngleSpeed = -1;
    		}
    		
    	} else { // if its within acceptable zone:
    		
    		neededAngleSpeed = 0; // sets the desired motor speed to 0  -  STOP
    	}
    	
    	SmartDashboard.putNumber("differenceToTargetInDegrees", differenceToTargetInDegrees);
    	SmartDashboard.putNumber("needeAngleSpeed", neededAngleSpeed);
    	SmartDashboard.putNumber("targetAngleInEncoderTicksFacingFront", targetAngle);
    	
    	angleMotorSpeedController.set(neededAngleSpeed); // sets the motor to the desired speed
    	
    	//ModuleAnglePIDController.setAngleSetpoint(targetAngle); // tells the PID what it needs to get to (in degrees)
    }
    
    public void setRotationSpeed (double targetRotationSpeed) { // receives target angle gets the module to that angle
    	angleMotorSpeedController.set(targetRotationSpeed); // sets the speed of the rotation motor to target rotation speed 
    }
    
    public void setDriveSpeed (double targetSpeed) { // receives target speed and gets the module to that speed
    	driveMotorSpeedController.set(targetSpeed); // sets the speed of the driveMotor to target speed
    }
    
}

