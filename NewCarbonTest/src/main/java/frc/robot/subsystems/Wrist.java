// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;

public class Wrist{
        /* Creates a new Wrist. */

      static double kP = 2;
      static double kI = 0;
      static double kD = 0;
      static double kF = 0;
      static double arbitraryFF = 0;

      public static int upperLimitEncoderCounts = 5632;
      public static int lowerLimitEncoderCounts = -682;
      public static int calibrationValue = 0;
      double encoderCountsPerAngle = 34.133;

      public static int controlMode = 1;
      static boolean limitDebounce = false;
      private TalonSRX wristMotor = new TalonSRX(Constants.wristMotor);

      private DigitalInput[] limitSwitches = {
        new DigitalInput(Constants.wristBottom),
        new DigitalInput(Constants.wristTop)
      };

      public Wrist(){
        wristMotor.configFactoryDefault();
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(true);
        wristMotor.setSensorPhase(false);
        wristMotor.configContinuousCurrentLimit(30);
        wristMotor.configPeakCurrentDuration(2000);
        wristMotor.enableCurrentLimit(true);
        wristMotor.configForwardSoftLimitEnable(false);
        wristMotor.configReverseSoftLimitEnable(false);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        wristMotor.config_kP(0, kP, 30);
        wristMotor.config_kI(0, kI, 30);
        wristMotor.config_kP(0, kP, 30);
        wristMotor.configClosedloopRamp(0.1, 100);
      }
    public double getPosition() {
        return wristMotor.getSelectedSensorPosition() + calibrationValue;
    } 
    
    public double getSelectedSensorVelocity(){
            return wristMotor.getSelectedSensorVelocity();
    }
        
    public ControlMode getTalonControlMode(){
        return wristMotor.getControlMode();
    }

    public boolean getEncoderHealthy(){
        return wristMotor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
    }
    
    public boolean getLimitSwitchState(int limitSwitchIndex){
        return !limitSwitches[limitSwitchIndex].get();
    }

    public void zeroEncoder(int mode){
        switch (mode) {
            case 2: 
                wristMotor.setSelectedSensorPosition(lowerLimitEncoderCounts, 0, 0);
                limitDebounce = true;
                    break;
            case 1:
                wristMotor.setSelectedSensorPosition(upperLimitEncoderCounts, 0, 0);
                limitDebounce = true;
                    break;
            case 0:
                limitDebounce = false;
                    break;

            default:
                limitDebounce = false;
                    break;
        }
    }
    
/*   public void zeroEncoder() {
        if(getLimitSwitchState(0)) {
            wristMotor.setSelectedSensorPosition(lowerLimitEncoderCounts, 0, 0);
            limitDebounce = true;
        } else if(getLimitSwitchState(1)) {
            wristMotor.setSelectedSensorPosition(upperLimitEncoderCounts, 0, 0);
            limitDebounce = true;
        } else
            limitDebounce = false;
    }
*/
    public void setEncoderPosition(int position) {
        wristMotor.setSelectedSensorPosition(position, 0, 0);
    }

    public double getAngle() {
        return getPosition() / encoderCountsPerAngle;
    }

    public void setDirectOutput(double output) {
        if (output == 0) {
            if(getEncoderHealthy())
                wristMotor.set(ControlMode.Position, getPosition(), DemandType.ArbitraryFeedForward, arbitraryFF);
            else
                wristMotor.set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, arbitraryFF);
        } else
            wristMotor.set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, arbitraryFF);
    }

    public void setIncrementedPosition(double angle) {
        double currentPosition = getPosition();
        double encoderCounts = angle * encoderCountsPerAngle + currentPosition;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

    }
    public void setAbsolutePosition(double angle) {
        double encoderCounts = angle * encoderCountsPerAngle;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

    }
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

}

