package org.usfirst.frc.team4786.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team4786.robot.subsystems.SwerveTrain;
import org.usfirst.frc.team4786.robot.Robot;

/**
 *
 */
public class ResetNavX extends InstantCommand {

    public ResetNavX() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	
    	Robot.swerveTrain.navX.reset();
    	
    }

}
