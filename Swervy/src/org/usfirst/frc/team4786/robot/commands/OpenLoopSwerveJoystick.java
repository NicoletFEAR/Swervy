package org.usfirst.frc.team4786.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team4786.robot.Robot;
import org.usfirst.frc.team4786.robot.RobotMap;
import org.usfirst.frc.team4786.robot.subsystems.SwerveTrain;

/**
 *
 */
public class OpenLoopSwerveJoystick extends Command {
	
    public OpenLoopSwerveJoystick() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.swerveTrain);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double intentAxesOpenLoop[] = Robot.swerveTrain.convertJoystickToIntentAxes();
    	double wheelAngleSpeedsArray[] = Robot.swerveTrain.convertIntentToWheelAngleSpeed(intentAxesOpenLoop[0], intentAxesOpenLoop[1], intentAxesOpenLoop[2]);
    	Robot.swerveTrain.setModuleAngleSpeeds(wheelAngleSpeedsArray);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
