
package org.usfirst.frc.team4786.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4786.robot.commands.*;
import org.usfirst.frc.team4786.robot.subsystems.*;
import org.usfirst.frc.team4786.robot.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static OI oi; // gives the robot an OI called oi
	
	public static ModuleDriver backRight = new ModuleDriver (RobotMap.backRightAngleMotorID, RobotMap.backRightDriveMotorID); // gives the robot our 4 swerve modules 
	public static ModuleDriver backLeft = new ModuleDriver (RobotMap.backLeftAngleMotorID, RobotMap.backLeftDriveMotorID); // ModuleDriver (int angleMotorSpeedControllerID, int driveMotorSpeedControllerID)
	public static ModuleDriver frontRight = new ModuleDriver (RobotMap.frontRightAngleMotorID, RobotMap.frontRightDriveMotorID);
	public static ModuleDriver frontLeft = new ModuleDriver (RobotMap.frontLeftAngleMotorID, RobotMap.frontLeftDriveMotorID);

	public static SwerveTrain swerveTrain = new SwerveTrain (backRight, backLeft, frontRight, frontLeft); // makes a SwerveTrain out of our 4 modules
	

	Command autonomousCommand; // autonomous stuff
	SendableChooser<Command> chooser = new SendableChooser<>(); // autonomous stuff

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() { // run when the robot turns on
		oi = new OI();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() { // run when the robot is first disabled... unreliable

	}

	@Override
	public void disabledPeriodic() { // happens while robot is disables... unreliable
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() { // run once at the start of autonomous
		autonomousCommand = chooser.getSelected(); // gets the selected auto command 

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null) // If the auto command seems good...
			autonomousCommand.start(); // run it!
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() { // called multiple times per second during autonomous
		Scheduler.getInstance().run(); // runs whatever commands are scheduled
	}

	@Override
	public void teleopInit() { // runs at the start of teleop
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel(); // ends the autonomous
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() { // runs repeatedly during teleop
		Scheduler.getInstance().run();
		
		//swerveDrive.drive (joystick.getRawAxis (0), joystick.getRawAxis (1), joystick.getRawAxis (2));

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
