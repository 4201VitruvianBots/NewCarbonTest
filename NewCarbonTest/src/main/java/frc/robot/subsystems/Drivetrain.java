// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
      

    final TalonSRX[] driveMotors = {
   		new TalonSRX(Constants.leftFrontDriveMotor),
    	new TalonSRX(Constants.leftRearDriveMotor),
    	new TalonSRX(Constants.rightFrontDriveMotor),
    	new TalonSRX(Constants.rightRearDriveMotor)

    }; 
    
    driveMotors[0].setInverted(true);
    driveMotors[1].setInverted(true);
    driveMotors[2].setInverted(false);
    driveMotors[3].setInverted(false);





  }

  public void setDriveStates(int mode){
    switch(mode) {
      case 2:  
        
        for(var motor : driveMotor)
            motor.setNeutralMode(NeutralMode.Coast);

        for(var breakMode : breakMode)
            breakMode = false;

          break;


      case 1:

        for(var motor : driveMotor)
            motor.setNeutralMode(NeutralMode.brake);

        for(var brakeMode : brakeMode)
        brakeMode = true;

        break;

      case 0:

      default:

        driveMotor[0].setNeutralMode(NeutralMode.brake);
        driveMotor[1].setNeutralMode(NeutralMode.brake);
        driveMotor[2].setNeutralMode(NeutralMode.brake);
        driveMotor[3].setNeutralMode(NeutralMode.brake);

        brakeMode[0] = true;
        brakeMode[1] = true;
        brakeMode[2] = true;
        brakeMode[3] = true;

       break;

      
    }



}

private void updateSmartDashboard() {
  if (RobotBase.isReal()) {
      SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
      SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
      SmartDashboardTab.putNumber("DriveTrain", "Left Distance", getWheelDistanceMeters(0));
      SmartDashboardTab.putNumber("DriveTrain", "Right Distance", getWheelDistanceMeters(2));
      SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
              Units.metersToFeet(getRobotPose().getTranslation().getX()));
      SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
              Units.metersToFeet(getRobotPose().getTranslation().getY()));
      SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPose().getRotation().getDegrees());
      SmartDashboardTab.putNumber("DriveTrain", "leftSpeed",
              Units.metersToFeet(getSpeeds().leftMetersPerSecond));
      SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
              Units.metersToFeet(getSpeeds().rightMetersPerSecond));
//            SmartDashboardTab.putBoolean("DriveTrain","high gear",
//                    getDriveShifterStatus());

      SmartDashboardTab.putNumber("Turret", "Robot Angle", getAngle());
  } else {
      SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
      SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
      SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
              Units.metersToFeet(getRobotPose().getTranslation().getX()));
      SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
              Units.metersToFeet(getRobotPose().getTranslation().getY()));
      SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPose().getRotation().getDegrees());
      SmartDashboardTab.putNumber("DriveTrain", "leftSpeed",
              Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
      SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
              Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));

      SmartDashboardTab.putNumber("Turret", "Robot Angle", getAngle());
  }
}


  @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));

        updateSmartDashboard();
      }

}

