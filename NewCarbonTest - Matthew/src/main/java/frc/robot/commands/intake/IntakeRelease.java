/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeRelease extends Command {
    int outtakeState;

    public IntakeRelease() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.wrist);
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // Read this initially to avoid flickering
        outtakeState = Intake.outtakeState;

        Intake.overridePassive = true;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        switch (outtakeState) {
            case 2:
                Robot.intake.setCargoIntakeOutput(RobotContainer.CARGO_OUTTAKE_SPEED);
                break;
            case 1:
                Robot.intake.setHatchGroundIntakeOutput(RobotContainer.HATCH_GROUND_OUTTAKE_SPEED);
                break;
            case 0:
            default:
                Robot.intake.setHatchIntakeOutput(RobotContainer.HATCH_OUTTAKE_SPEED);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        switch (outtakeState) {
            case 2:
            case 1:
                Robot.intake.setCargoIntakeOutput(0);
                break;
            case 0:
            default:
                Timer.delay(0.5);
                Robot.intake.setHatchIntakeOutput(0);
                break;
        }

        Intake.overridePassive = false;
    }

    @Override
    protected void interrupted() {
        end();
    }
}
