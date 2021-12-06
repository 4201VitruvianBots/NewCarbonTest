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

public class ManualWristUp extends CommandBase {
  /**
   * Creates a new ManualWristUp.
   */
  public ManualWristUp() {
    addRequirements(RobotContainer.wrist);
   // setTimeout(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     RobotContainer.wrist.setDirectOutput(0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(RobotContainer.wrist.getLimitSwitchState(1)) {
      Wrist.controlMode = 1;
  }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return/* isTimedOut() ||*/ RobotContainer.wrist.getLimitSwitchState(1);
  }
}
