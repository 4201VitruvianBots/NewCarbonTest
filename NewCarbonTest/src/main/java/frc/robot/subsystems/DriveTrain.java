// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  DoubleSolenoid driveTrainShifters = new DoubleSolenoid(Constants.pcmOne,
  Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);
  public AHRS navX = new AHRS(SerialPort.Port.kMXP);

  public static int controlMode = 0;


  public DriveTrain() {
    for (TalonSRX motor : driveMotors) {
      motor.configFactoryDefault();
      motor.config_kP(0, 0.25, 30);
      motor.config_kI(0, 0, 30);
      motor.config_kD(0, 10, 30);
      motor.config_kF(0, 1023.0 / 72000.0, 30);
      motor.configVoltageCompSaturation(12);
      motor.enableVoltageCompensation(true);
      motor.configContinuousCurrentLimit(30);
      motor.configPeakCurrentLimit(40);
      motor.configPeakCurrentDuration(2000);
      motor.enableCurrentLimit(true);
      motor.configOpenloopRamp(0.6);
      motor.setNeutralMode(NeutralMode.Coast);
  }

  driveMotors[0].setInverted(true);
  driveMotors[1].setInverted(true);
  driveMotors[2].setInverted(false);
  driveMotors[3].setInverted(false);

  driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 
//making rightback follow right front and vice versa
  driveMotors[1].follow(driveMotors[0]);
  driveMotors[3].follow(driveMotors[2]);
  }

//method to give a voltage value to motors
public static void setMotorPercentOutputs(double leftoutput, double rightoutput) {
driveMotors[0].set(ControlMode.PercentOutput, leftoutput);
driveMotors[2].set(ControlMode.PercentOutput, rightoutput);
}

public void setMotorArcadeDrive(double throttle, double turn) {
  double leftPWM = throttle + turn;
  double rightPWM = throttle - turn;
  
  setMotorPercentOutputs(leftPWM, rightPWM);
}

public void setMotorTankDrive(double leftOutput, double rightOutput){
setMotorPercentOutputs(leftOutput, rightOutput);
}
//if true then motors will be in coast mode, if false then motors will be in break mode 
public void setDriveMotorsState(boolean state) {
  for (TalonSRX driveMotor : driveMotors)
      driveMotor.setNeutralMode((state) ? NeutralMode.Coast : NeutralMode.Brake);
}

public void setArcadeDriveVelocity(double throttle, double turn) {
  double leftPWM = throttle + turn;
  double rightPWM = throttle - turn;

  if (rightPWM > 1.0) {
      leftPWM -= rightPWM - 1.0;
      rightPWM = 1.0;
  } else if (rightPWM < -1.0) {
      leftPWM -= rightPWM + 1.0;
      rightPWM = -1.0;
  } else if (leftPWM > 1.0) {
      rightPWM -= leftPWM - 1.0;
      leftPWM = 1.0;
  } else if (leftPWM < -1.0) {
      rightPWM -= leftPWM + 1.0;
      leftPWM = -1.0;
  }

  setMotorVelocityOutput(leftPWM, rightPWM);
}

public boolean getDriveShifterStatus() {
        return (driveTrainShifters.get() == DoubleSolenoid.Value.kForward) ? true : false;
}


public void setDriveShifterStatus(boolean state) {
    driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
}

public void setMotorVelocityOutput(double leftOutput, double rightOutput) {
  //TODO: Update values to match robot with full load.
  double k_maxVelocity = getDriveShifterStatus() ? 8789 : 18555;  // in encoder units/sec

  // TODO: Normalize this
  double leftVelocity = leftOutput * k_maxVelocity;
  double rightVelocity = rightOutput * k_maxVelocity;

  leftVelocity = (leftVelocity > k_maxVelocity) ? k_maxVelocity : (leftVelocity < -k_maxVelocity) ? -k_maxVelocity: leftVelocity;
  rightVelocity = (rightVelocity > k_maxVelocity) ? k_maxVelocity : (rightVelocity < -k_maxVelocity) ? -k_maxVelocity: rightVelocity;

  driveMotors[0].set(ControlMode.Velocity, leftVelocity);
  driveMotors[2].set(ControlMode.Velocity, rightVelocity);
}

public void updateSmartDashboard() {
SmartDashboard.putNumber("Robot Angle", navX.getAngle());
}

public void updateShuffleboard() {
SmartDashboard.putNumber("Robot Angle", navX.getAngle());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
