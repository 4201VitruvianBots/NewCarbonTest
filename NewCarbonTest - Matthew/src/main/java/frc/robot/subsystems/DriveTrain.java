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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  private TalonSRX[] driveMotors = {
    new TalonSRX(Constants.leftFrontDriveMotor),
    new TalonSRX(Constants.leftRearDriveMotor),
    new TalonSRX(Constants.rightFrontDriveMotor),
    new TalonSRX(Constants.rightRearDriveMotor),
  };

  // PID controller values
  public double kP = 1.33;
  public double kI = 0;
  public double kD = 0;

  // SVA values 
  private double kS = 0.19;
  private double kV = 2.23;
  private double kA = 0.0289;

  // Set up constants
  private double wheelDiameter = 0.5;
  private final double gearRatio = 1.0 / 5.0;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    DifferentialDriveOdometry odometry;
    DifferentialDrivePoseEstimator m_poseEstimator;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    PIDController leftPIDController = new PIDController(kP, kI, kD);
    PIDController rightPIDController = new PIDController(kP, kI, kD);

    PowerDistributionPanel m_pdp;
    
    double m_leftOutput, m_rightOutput;

    private final boolean[] brakeMode = {
            true,
            false,
            true,
            false
    };

    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    // Temporary until CTRE supports FalconFX in WPILib Sim
    private final TalonSRX[] simMotors =  new TalonSRX[4];

    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private ADXRS450_GyroSim m_gyroAngleSim;

    private DoubleSupplier joystickCorrector = null; // A turning joystick to adjust trajectories during tele-op

    public DriveTrain(PowerDistributionPanel pdp) {
        // Set up DriveTrain motors
        configureCtreMotors(driveMotors);

        m_pdp = pdp;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        if (RobotBase.isSimulation()) { // If our robot is simulated
            for(int i = 0; i < 4; i++)
                simMotors[i] = new TalonSRX(24 + i);
            configureCtreMotors(simMotors);
            simMotors[0].setSensorPhase(true);
            simMotors[2].setSensorPhase(false);

            m_drivetrainSimulator = new DifferentialDrivetrainSim(
                    DriveConstants.kDrivetrainPlant,
                    DriveConstants.kDriveGearbox,
                    DriveConstants.kDriveGearingHigh,
                    DriveConstants.kTrackwidthMeters,
                    Constants.DriveConstants.kWheelDiameterMeters / 2.0,
                    null);

            m_gyroAngleSim = new ADXRS450_GyroSim(m_gyro);
        }
        SmartDashboard.putData("DT Subsystem", this);
    }

    public void configureCtreMotors(BaseTalon... motors) {
        for(int i = 0; i < motors.length; i++) {
            motors[i].configFactoryDefault();
            motors[i].configOpenloopRamp(0.1);
            motors[i].configClosedloopRamp(0.1);
            motors[i].setNeutralMode(NeutralMode.Coast);
            motors[i].configForwardSoftLimitEnable(false);
            motors[i].configReverseSoftLimitEnable(false);

            if(motors[i] instanceof TalonSRX) {
                driveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
                driveMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            } else if(motors[i] instanceof TalonSRX) {
                simMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
                simMotors[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            }
        }

        motors[0].setInverted(true);
        motors[1].setInverted(true);
        motors[2].setInverted(false);
        motors[3].setInverted(false);

        motors[0].setSensorPhase(false);
        motors[2].setSensorPhase(false);

        motors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        motors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());
        motors[1].setNeutralMode(NeutralMode.Brake);
        motors[3].setNeutralMode(NeutralMode.Brake);

        motors[1].configOpenloopRamp(0);
        motors[3].configOpenloopRamp(0);
    }

    // Self-explanatory functions

    public double getEncoderCount(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorPosition();
    }

    public double getAngle() {
        if(RobotBase.isReal())
            return navX.getAngle();
        else
            return m_gyro.getAngle();
    }

    public double getHeading() {
        if(RobotBase.isReal())
            return Math.IEEEremainder(-navX.getAngle(), 360);
        else
            return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetAngle() {
        navX.zeroYaw();
    }

    public void setNavXOffset(double angle) { // Angle in degrees
        navX.setAngleAdjustment(angle);
    }

    public double getWheelDistanceMeters(int sensorIndex) {

        if(RobotBase.isReal())
            return (driveMotors[sensorIndex].getSelectedSensorPosition() / 4096.0) * gearRatio * Math.PI * Units.feetToMeters(wheelDiameter);
        else {
            return (simMotors[sensorIndex].getSelectedSensorPosition() / 4096.0) * Math.PI * Units.feetToMeters(wheelDiameter);
            }
    }

    // ???
    public double getMotorInputCurrent(int motorIndex) {
        return driveMotors[motorIndex].getSupplyCurrent();
    }

    public void resetEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
        if(RobotBase.isSimulation()) {
            simMotors[0].getSimCollection().setQuadratureRawPosition(0);
            simMotors[2].getSimCollection().setQuadratureRawPosition(0);
          }
        }

        public void setAutosJoystick(DoubleSupplier supplier) {
          joystickCorrector = supplier;
        }

        public void setMotorArcadeDrive(double throttle, double turn) {
          double leftPWM = throttle + turn;
          double rightPWM = throttle - turn;

          // Normalization
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
          if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage)) > batteryVoltage) {
            leftVoltage *= batteryVoltage / 12.0;
            rightVoltage *= batteryVoltage / 12.0;
          }
          if (joystickCorrector == null) {
            setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);
          } else {
            setMotorPercentOutput(leftVoltage / batteryVoltage + joystickCorrector.getAsDouble(),
                rightVoltage / batteryVoltage - joystickCorrector.getAsDouble());
          }
        }

        private void setMotorPercentOutput(double leftOutput, double rightOutput) {
          m_leftOutput = leftOutput;
          m_rightOutput = rightOutput;
          driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
          driveMotors[2].set(ControlMode.PercentOutput, rightOutput);

          if (RobotBase.isSimulation()) {
            simMotors[0].set(ControlMode.PercentOutput, leftOutput);
            simMotors[2].set(ControlMode.PercentOutput, rightOutput);
          }
        }

        // Make motors neutral
        public void setDriveTrainNeutralMode(int mode) {
          switch (mode) {
            case 2:
              for (var motor : driveMotors)
                motor.setNeutralMode(NeutralMode.Coast);
              for (var brakeMode : brakeMode)
                brakeMode = false;
              break;
            case 1:
              for (var motor : driveMotors)
                motor.setNeutralMode(NeutralMode.Brake);
              for (var brakeMode : brakeMode)
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

        public DifferentialDriveWheelSpeeds getSpeeds() {
          double leftMetersPerSecond = 0, rightMetersPerSecond = 0;

          if (RobotBase.isReal()) {
            leftMetersPerSecond = (driveMotors[0].getSelectedSensorVelocity() * 10.0 / 2048.0) * gearRatio * Math.PI
                * Units.feetToMeters(wheelDiameter);
            rightMetersPerSecond = (driveMotors[2].getSelectedSensorVelocity() * 10.0 / 2048.0) * gearRatio * Math.PI
                * Units.feetToMeters(wheelDiameter);
          }

          return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
        }

        public double getTravelDistance() {
          double leftMeters, rightMeters;

          if (RobotBase.isReal()) {
            leftMeters = (driveMotors[0].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI
                * Units.feetToMeters(wheelDiameter);
            rightMeters = (driveMotors[2].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI
                * Units.feetToMeters(wheelDiameter);
            return (leftMeters + rightMeters) / 2.0;
          } else {
            leftMeters = (simMotors[0].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI
                * Units.feetToMeters(wheelDiameter);
            rightMeters = (simMotors[2].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI
                * Units.feetToMeters(wheelDiameter);
            return (leftMeters + rightMeters) / 2.0;
          }
        }

        public SimpleMotorFeedforward getFeedforward() {
          return feedforward;
        }

        public Pose2d getRobotPose() {
          return odometry.getPoseMeters();
        }

        public DifferentialDriveKinematics getDriveTrainKinematics() {
          return kinematics;
        }

        public PIDController getLeftPIDController() {
          return leftPIDController;
        }

        public PIDController getRightPIDController() {
          return rightPIDController;
        }

        public void resetOdometry(Pose2d pose, Rotation2d rotation) {
          if (RobotBase.isSimulation()) {
            resetEncoderCounts();
            m_drivetrainSimulator.setPose(pose);
          }
          setNavXOffset(rotation.getDegrees());
          odometry.resetPosition(pose, rotation);
          resetEncoderCounts();
        }

        @Override
        public void periodic() {
          // This method will be called once per scheduler run
          odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));
        }

        public double getDrawnCurrentAmps() {
          return m_drivetrainSimulator.getCurrentDrawAmps();
        }

        double maxVel;

        @Override
        public void simulationPeriodic() {
          m_drivetrainSimulator.setInputs(m_leftOutput * RobotController.getBatteryVoltage(),
              m_rightOutput * RobotController.getBatteryVoltage());
          m_drivetrainSimulator.update(0.040);

          // For CTRE devices, you must call this function periodically for simulation
          Unmanaged.feedEnable(40);
          simMotors[0].getSimCollection()
              .setQuadratureRawPosition(distanceMetersToTalonSrxUnits(m_drivetrainSimulator.getLeftPositionMeters()));
          simMotors[0].getSimCollection().setQuadratureVelocity(
              velocityMetersToTalonSrxUnits(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
          simMotors[2].getSimCollection()
              .setQuadratureRawPosition(distanceMetersToTalonSrxUnits(m_drivetrainSimulator.getRightPositionMeters()));
        simMotors[2].getSimCollection().setQuadratureVelocity(velocityMetersToTalonSrxUnits(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));
        m_gyroAngleSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    int distanceMetersToTalonSrxUnits(double meters) {
        // To simplify, for simulating Talons, pretend they are on the wheel axel (e.g. don't care about the gear ratio)
        return (int) (meters * 4096.0 / (Units.feetToMeters(wheelDiameter) * Math.PI));
    }

    int velocityMetersToTalonSrxUnits(double meters) {
        // To simplify, for simulating Talons, pretend they are on the wheel axel (e.g. don't care about the gear ratio)
        return (int) (meters * 4096.0 / (Units.feetToMeters(wheelDiameter) * Math.PI * 10.0));
    }

    int distanceMetersToFalconFxUnits(double meters) {
//        double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;

        return (int) (meters * 2048.0 / (10 * gearRatio * Math.PI));
    }
}