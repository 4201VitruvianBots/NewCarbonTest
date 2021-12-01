/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.constants.Constants;

/**
 * An example command.  You can replace me with your own command.
 */
public class HoldHatchIntakeIntake extends Command {
    public HoldHatchIntakeIntake() {
        // Use requires() here to declare subsystem dependencies
        requires(RobotContainer.intake);
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

    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
                break;
            case 0:
            default:
                RobotContainer.intake.setHatchIntakeOutput(Constants.Setpoints.HATCH_INTAKE_SPEED);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
            case 0:
            default:
                Timer.delay(0.25);
                RobotContainer.intake.setHatchIntakeOutput(0);
                break;
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
