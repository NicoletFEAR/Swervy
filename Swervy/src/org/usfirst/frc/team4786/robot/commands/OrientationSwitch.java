package org.usfirst.frc.team4786.robot.commands;

import org.usfirst.frc.team4786.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class OrientationSwitch extends InstantCommand {

    public OrientationSwitch() {
        super();
        // Use requires() here to declare subsystem dependencies
        requires(Robot.swerveTrain);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.swerveTrain.switchControlModeOrientation(); // switches between robot and field oriented drive
    }

}
