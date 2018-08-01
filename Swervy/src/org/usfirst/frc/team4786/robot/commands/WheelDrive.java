package org.usfirst.frc.team4786.robot.commands;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WheelDrive {
	
	private final double MAX_VOLTS = 4.95;

	private WPI_TalonSRX angleMotor;
    private WPI_TalonSRX speedMotor;
//    public static PIDController pidController; 
    private SensorCollection encoder;
    
    public WheelDrive (int angleMotor, int speedMotor) {
        this.angleMotor = new WPI_TalonSRX (angleMotor);
        this.speedMotor = new WPI_TalonSRX (speedMotor);
        
        this.encoder = this.angleMotor.getSensorCollection();

//        pidController = new PIDController (1, 0, 0, this.angleMotor, this.angleMotor);
//        pidController.setOutputRange (-1, 1);
//        pidController.setContinuous ();
//        pidController.enable ();
    }
    
    public void drive (double speed, double angle) {
        speedMotor.set(speed);
        angleMotor.set(speed);

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
