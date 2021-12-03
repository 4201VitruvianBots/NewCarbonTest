/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    /**
     * Creates a new Intake.
     */
    public static int intakeState = 0;
    public static int outtakeState = 0;

    public boolean[] intakeIndicator = { false, false, false };
    public static boolean overridePassive = false;
    static boolean isBannerTripped = false;

    private final TalonSRX[] intakeMotors = { new TalonSRX(Constants.cargoIntakeMotor),
            new TalonSRX(Constants.hatchIntakeMotor) };

    DigitalInput BannerIR = new DigitalInput(Constants.bannerIR);

    public Intake() {
        for (TalonSRX intakeMotor : intakeMotors) {
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
        }
        intakeMotors[0].setInverted(true);
        intakeMotors[1].setInverted(false);
    }

    public void setCargoIntakeOutput(double output) {
        intakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setHatchGroundIntakeOutput(double output) {
        intakeMotors[0].set(ControlMode.PercentOutput, -output);
    }

    public void setHatchIntakeOutput(double output) {
        intakeMotors[1].set(ControlMode.PercentOutput, output);
    }

    public void updateIntakeIndicator() {
        for (int i = 0; i < intakeIndicator.length; i++)
            intakeIndicator[i] = false;
        intakeIndicator[intakeState] = true;
    }

    public void updateCargoIntakeState(boolean buttonPress) {
        if (buttonPress) {

        } else if (BannerIR.get() && !isBannerTripped) {
            setCargoIntakeOutput(Constants.CARGO_HOLD_SPEED);
        } else if (isBannerTripped) {
            setCargoIntakeOutput(0);
            isBannerTripped = false;
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Cargo", intakeIndicator[2]);
        SmartDashboard.putBoolean("Hatch", intakeIndicator[0]);
        SmartDashboard.putBoolean("Banner IR", BannerIR.get());
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
