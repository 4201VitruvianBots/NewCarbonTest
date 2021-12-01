/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetIntakeState extends InstantCommand {
    int state;
    public SetIntakeState(int state) {
        requires(RobotContainer.intake);
        requires(RobotContainer.intakeExtend);
        this.state = state;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if(state == 2) {
            RobotContainer.intake.setHatchIntakeOutput(0);
            RobotContainer.intakeExtend.setHarpoonExtend(false);
        }

        if(state != 2)
            RobotContainer.intake.setCargoIntakeOutput(0);

        Intake.intakeState = state;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }
}
