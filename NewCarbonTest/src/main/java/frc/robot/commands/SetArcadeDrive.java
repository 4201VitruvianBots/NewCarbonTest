// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class SetArcadeDrive extends CommandBase {
    private DoubleSupplier m_throttle;
    private DriveTrain m_driveTrain;
    private DoubleSupplier m_turn;
  /** Creates a new SetArcadeDrive. */
  public SetArcadeDrive(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn)
   {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_throttle = throttle;
    m_turn = turn;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickY = (Math.abs(m_throttle.getAsDouble()) > 0.05) ? m_throttle.getAsDouble() : 0;
    double joystickX = (Math.abs(m_turn.getAsDouble()) > 0.05) ? m_turn.getAsDouble() : 0;
    double throttle = joystickY;
    double turn = joystickX;

    throttle = throttle < 0 ? Math.max(-0.7, throttle) : throttle;
    m_driveTrain.setMotorArcadeDrive(throttle, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
