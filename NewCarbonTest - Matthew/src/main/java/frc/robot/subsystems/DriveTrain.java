// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonSRX[] driveMotors = {
    new TalonSRX(Constants.leftFrontDriveMotor),
    new TalonSRX(Constants.leftRearDriveMotor),
    new TalonSRX(Constants.rightFrontDriveMotor),
    new TalonSRX(Constants.rightRearDriveMotor),
  };

  private boolean[] brakeMode = {
    true,
    false,
    true,
    false
  };

  // PID controller values (InfiniteRecharge2020)
  public double kP = 1.33;
  public double kI = 0;
  public double kD = 0;

  // SVA values (InfiniteRecharge2020)
  private double kS = 0.19;
  private double kV = 2.23;
  private double kA = 0.0289;

  // Set up constants (InfiniteRecharge2020)
  private double gearRatioLow = 1 / 14.14;
  private double gearRatioHigh = 1 / 7.49;
  private double wheelDiameter = 0.5;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
