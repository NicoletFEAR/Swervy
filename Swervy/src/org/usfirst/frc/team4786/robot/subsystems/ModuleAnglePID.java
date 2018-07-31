package org.usfirst.frc.team4786.robot.subsystems;

import org.usfirst.frc.team4786.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class ModuleAnglePID extends PIDSubsystem { // PID that runs in the background and will make our modules set their angles
	
	public static SensorCollection angleEncoder;
	public static WPI_TalonSRX angleTalon;
    // Initialize your subsystem here
    public ModuleAnglePID(ModuleDriver thisSwerveModule) {
    	
    	super("ModuleAnglePID", 2.0, 0.0, 0.0);
    	setAbsoluteTolerance(0.05);
    	getPIDController().setContinuous(false);
    	
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        //enable(); - Enables the PID controller.
    	angleEncoder = thisSwerveModule.angleEncoder;
    	angleTalon = thisSwerveModule.angleMotorSpeedController;
    	
    	setInputRange(-(RobotMap.encoderCodesPerRev / 2), (RobotMap.encoderCodesPerRev / 2)); // input range (- halfway to + halfway of encoder ticks)
    	setOutputRange(-1, 1); // output range. we want speeds of motor (-1-, 1)
    	
    	enable();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return angleEncoder.getQuadraturePosition();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	angleTalon.set(output); // sets the talon to the speed the PID controller outputs
    }
    
    public void setAngleSetpoint(double targetAngleSetpoint) {
    	// (may have to do some math to turn the angle into and encoder value)
    	// targetAngleSetpoint will be an angle in degrees that we want to reach. 
    	// It should be between -180 and 180.
    	// We need to set the Setpoint to be that number but in encoder ticks (-512, 512), with 0 being front
    	double encoderTargetAngleSetpoint = targetAngleSetpoint * (RobotMap.encoderCodesPerRev / 360);
    	
    	setSetpoint(encoderTargetAngleSetpoint);
    }
    
    
}
