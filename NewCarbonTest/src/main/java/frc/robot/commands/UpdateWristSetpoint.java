/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;

public class UpdateWristSetpoint extends CommandBase {
  /**
   * Creates a new UpdateWristSetpoint.
   */
  public UpdateWristSetpoint() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickOutput = RobotContainer.getXBoxRightY();
//TODO: Change to switch statement if possible
    if (Wrist.controlMode == 1) {
        if(Math.abs(joystickOutput) > 0.05) {
            double setpoint = joystickOutput * 10;

            // TODO: Change this logic to use limit switches when they are fixed
            if(setpoint <= 0 && RobotContainer.wrist.getAngle() < 0.1 || setpoint >= 120  && RobotContainer.wrist.getAngle() > 119.9)
                RobotContainer.enableXBoxRumbleTimed(0.2);

            RobotContainer.wrist.setIncrementedPosition(setpoint);
        }
    } else {
        if(Math.abs(joystickOutput) > 0.05)
            RobotContainer.wrist.setDirectOutput (joystickOutput);
        else
            RobotContainer.wrist.setDirectOutput (0);
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
