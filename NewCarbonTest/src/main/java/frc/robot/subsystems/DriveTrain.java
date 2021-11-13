// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  static TalonSRX[] driveMotors = {
    new TalonSRX(Constants.leftFrontDriveMotor),
    new TalonSRX(Constants.leftRearDriveMotor),
    new TalonSRX(Constants.rightFrontDriveMotor),
    new TalonSRX(Constants.rightRearDriveMotor),
  };

  
  public DriveTrain() {
//making rightback follow right front and vice versa
  driveMotors[2].follow(driveMotors[1]);
  driveMotors[4].follow(driveMotors[1]);

  DoubleSolenoid shifters = new DoubleSolenoid(Constants.pcm,
  Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);
  }
//method to give a voltage value to motors
public static void SetMotorPercentOutputs(double leftoutput, double rightoutput) {
driveMotors[1].set(ControlMode.PercentOutput, leftoutput);
driveMotors[4].set(ControlMode.PercentOutput, rightoutput);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
