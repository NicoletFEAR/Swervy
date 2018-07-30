package org.usfirst.frc.team4786.robot.commands;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;

public class WheelDrive {
	
	private final double MAX_VOLTS = 4.95;

	public static WPI_TalonSRX angleMotor;
    public static WPI_TalonSRX speedMotor;
//    public static PIDController pidController; 
    public static SensorCollection encoder;
    
    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new WPI_TalonSRX (angleMotor);
        this.speedMotor = new WPI_TalonSRX (speedMotor);
        
        this.encoder = this.angleMotor.getSensorCollection();

//        pidController = new PIDController (1, 0, 0, this.angleMotor, this.angleMotor);
//        pidController.setOutputRange (-1, 1);
//        pidController.setContinuous ();
//        pidController.enable ();
    }
    
    public void drive (double speed, double angle) {
        speedMotor.set (speed);

//        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
//        if (setpoint < 0) {
//            setpoint = MAX_VOLTS + setpoint;
//        }
//        if (setpoint > MAX_VOLTS) {
//            setpoint = setpoint - MAX_VOLTS;
//        }
//
//        pidController.setSetpoint (setpoint);
    }


}
