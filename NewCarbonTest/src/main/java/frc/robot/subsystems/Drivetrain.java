// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /* Creates a new ExampleSubsystem. */
  DifferentialDriveOdometry odometry;
  private final TalonSRX[] driveMotors = {
        new TalonSRX(Constants.leftFrontDriveMotor),
        new TalonSRX(Constants.leftRearDriveMotor),
        new TalonSRX(Constants.rightFrontDriveMotor),
        new TalonSRX(Constants.rightRearDriveMotor)
  };

  PowerDistributionPanel m_pdp;

  double m_leftOutput, m_rightOutput;

  private final boolean[] brakeMode = {
        true,
        false,
        true,
        false
};



    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

//    // The left-side drive encoder
//    private final Encoder m_leftEncoder =
//            new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0],
//                    Constants.DriveConstants.kLeftEncoderPorts[1],
//                    Constants.DriveConstants.kLeftEncoderReversed);
//
//    // The right-side drive encoder
//    private final Encoder m_rightEncoder =
//            new Encoder(Constants.DriveConstants.kRightEncoderPorts[0],
//                    Constants.DriveConstants.kRightEncoderPorts[1],
//                    Constants.DriveConstants.kRightEncoderReversed);
//    private EncoderSim m_leftEncoderSim;
//    private EncoderSim m_rightEncoderSim;

    // Temporary until CTRE supports FalconFX in WPILib Sim
    private final TalonSRX[] simMotors =  new TalonSRX[4];

    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private ADXRS450_GyroSim m_gyroAngleSim;

    private DoubleSupplier joystickCorrector = null;

        public DriveTrain() {

                

                final TalonSRX[] driveMotors = { new TalonSRX(Constants.leftFrontDriveMotor),
                                new TalonSRX(Constants.leftRearDriveMotor),
                                new TalonSRX(Constants.rightFrontDriveMotor),
                                new TalonSRX(Constants.rightRearDriveMotor)

                };

                driveMotors[0].setInverted(true);
                driveMotors[1].setInverted(true);
                driveMotors[2].setInverted(false);
                driveMotors[3].setInverted(false);

        }

        public void setMotorArcadeDrive(double throttle, double turn) {
                double leftPWM = throttle + turn;
                double rightPWM = throttle - turn;

                        double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
                        if (magnitude > 1.0) {
                        leftPWM *= 1.0 / magnitude;
                        rightPWM *= 1.0 / magnitude;
                        }
                       setMotorPercentOutput(leftPWM, rightPWM);

        }

        public void setMotorTankDrive(double leftOutput, double rightOutput) {
                setMotorPercentOutput(leftOutput, rightOutput);
            }
        
            public void setVoltageOutput(double leftVoltage, double rightVoltage) {
                var batteryVoltage = RobotController.getBatteryVoltage();
                if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage))
                        > batteryVoltage) {
                    leftVoltage *= batteryVoltage / 12.0;
                    rightVoltage *= batteryVoltage / 12.0;
                }
                
                if (joystickCorrector == null) {   
                    setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);
                } else {
                    setMotorPercentOutput(leftVoltage / batteryVoltage + joystickCorrector.getAsDouble(), rightVoltage / batteryVoltage - joystickCorrector.getAsDouble());
                }
            }
        
            private void setMotorPercentOutput(double leftOutput, double rightOutput) {
                m_leftOutput = leftOutput;
                m_rightOutput = rightOutput;
                driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
                driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
        
            }
        
            // Make motors neutral
            public void setDriveTrainNeutralMode(int mode) {
                switch(mode) {
                    case 2:
                        for(var motor : driveMotors)
                            motor.setNeutralMode(NeutralMode.Coast);
                        for(var brakeMode : brakeMode)
                            brakeMode = false;
                        break;
                    case 1:
                        for(var motor : driveMotors)
                            motor.setNeutralMode(NeutralMode.Brake);
                        for(var brakeMode : brakeMode)
                            brakeMode = true;
                        break;
                    case 0:
                    default:
                        driveMotors[0].setNeutralMode(NeutralMode.Brake);
                        driveMotors[1].setNeutralMode(NeutralMode.Coast);
                        driveMotors[2].setNeutralMode(NeutralMode.Brake);
                        driveMotors[3].setNeutralMode(NeutralMode.Coast);
                        brakeMode[0] = true;
                        brakeMode[1] = false;
                        brakeMode[2] = true;
                        brakeMode[3] = false;
                        break;
                }
            }
        
        //    public boolean getDriveShifterStatus() {
        //        return m_driveShifterState;
        //    }
        
        //    public void setDriveShifterStatus(boolean state) {
        //        m_driveShifterState = state;
        //        double gearRatio = state ? Constants.DriveConstants.kDriveGearingHigh : Constants.DriveConstants.kDriveGearingLow ;
        //        double kEncoderDistancePerPulse = state ? Constants.DriveConstants.kEncoderDistancePerPulseHigh : Constants.DriveConstants.kEncoderDistancePerPulseLow;
        
        //        m_drivetrainSimulator.setCurrentGearing(gearRatio);
        //        m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
        //        m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
        
        //        driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        //    }
        
            public DifferentialDriveWheelSpeeds getSpeeds() {
        //        double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;
        //        double gearRatio = gearRatioHigh;
                double leftMetersPerSecond = 0, rightMetersPerSecond = 0;
        
                
                return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
            }
        
        
                double leftMeters, rightMeters;
        
               
        
        
       
        
          
        
           
            
            
        
     
        
            
        

               



  

    
       
        








  @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
      }

}

