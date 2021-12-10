/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;

public class ToggleWristState extends CommandBase {
  /**
   * Creates a new ToggleWristState.
   */
  public ToggleWristState() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
        withTimeout(0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Scheduler.getInstance().removeAll();
    if(Wrist.controlMode == 1)
        Wrist.controlMode = 0;
    else {
        RobotContainer.wrist.setEncoderPosition(Wrist.upperLimitEncoderCounts);
        Wrist.controlMode = 1;
    }
    RobotContainer.enableXBoxRumbleTimed(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
