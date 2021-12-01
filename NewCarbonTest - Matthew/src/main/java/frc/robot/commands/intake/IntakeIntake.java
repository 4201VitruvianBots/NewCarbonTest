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
public class IntakeIntake extends Command {

    public IntakeIntake() {
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

        Intake.overridePassive = true;
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
                RobotContainer.intake.setCargoIntakeOutput(Constants.Setpoints.CARGO_INTAKE_SPEED);
                break;
            case 1:
                RobotContainer.intake.setHatchGroundIntakeOutput(Constants.Setpoints.HATCH_GROUND_INTAKE_SPEED);
                break;
            case 0:
            default:
                RobotContainer.intake.setHatchIntakeOutput(Constants.Setpoints.HATCH_INTAKE_SPEED);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if(Intake.intakeState == 2)
            return RobotContainer.intake.bannerIR.get();
        else
            return false;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        switch (Intake.intakeState) {
            case 2:
                break;
            case 1:
                RobotContainer.intake.setHatchIntakeOutput(Constants.Setpoints.HATCH_GROUND_HOLD_SPEED);
                break;
            case 0:
            default:
                Timer.delay(0.25);
                RobotContainer.intake.setHatchIntakeOutput(0);
                break;
        }

        Intake.overridePassive = false;
    }

    @Override
    protected void interrupted() {
        end();
    }
}
