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
public class IntakePassive extends Command {
    boolean isBannerTripped = false;

    public IntakePassive() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
            case 0:
            default:
                break;
        }
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
                break;
            case 1:
                Robot.intake.setHatchGroundIntakeOutput(RobotContainer.HATCH_GROUND_INTAKE_SPEED);
                break;
            case 0:
            default:
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
        switch (Intake.intakeState) {
            case 2:
                break;
            case 1:
                break;
            case 0:
            default:
                break;
        }
    }

    @Override
    protected void interrupted() {
        end();
    }
}
