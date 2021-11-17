// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  DoubleSolenoid driveTrainShifters = new DoubleSolenoid(Constants.pcm,
  Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);
  public AHRS navX = new AHRS(SerialPort.Port.kMXP);



  public DriveTrain() {
//making rightback follow right front and vice versa
  driveMotors[1].follow(driveMotors[0]);
  driveMotors[3].follow(driveMotors[2]);
  }

//method to give a voltage value to motors
public static void setMotorPercentOutputs(double leftoutput, double rightoutput) {
driveMotors[1].set(ControlMode.PercentOutput, leftoutput);
driveMotors[4].set(ControlMode.PercentOutput, rightoutput);
}
public void setMotorArcadeDrive(double throttle, double turn) {
  double leftPWM = throttle + turn;
  double rightPWM = throttle - turn;
  
  setMotorPercentOutputs(leftPWM, rightPWM);
}
public void setMotorTankDrive(double leftOutput, double rightOutput){
setMotorPercentOutputs(leftOutput, rightOutput);
}
public void setDriveMotorsState(boolean state) {
  for (TalonSRX driveMotor : driveMotors)
      driveMotor.setNeutralMode((state) ? NeutralMode.Coast : NeutralMode.Brake);
}
public void updateSmartDashboard() {
SmartDashboard.putNumber("Robot Angle", navX.getAngle());
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
