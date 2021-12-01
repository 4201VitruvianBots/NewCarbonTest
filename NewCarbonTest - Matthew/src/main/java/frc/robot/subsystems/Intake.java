/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class Intake extends Subsystem {

    public static int intakeState = 0;
    public static int outtakeState = 0;

    public boolean[] intakeIndicator = {false, false, false};

    public static boolean overridePassive = false;
    static boolean isBannerTripped = false;

    private TalonSRX[] intakeMotors = {
        new TalonSRX(Constants.CAN.cargoIntakeMotor),
        new TalonSRX(Constants.CAN.hatchIntakeMotor)
    };

    public DigitalInput bannerIR = new DigitalInput(Constants.DIOS.bannerIR);

    public Intake() {
        super("Intake");

        for(TalonSRX intakeMotor:intakeMotors) {
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
        }
        intakeMotors[0].setInverted(true);
        intakeMotors[1].setInverted(false);
    }

    public void setCargoIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setHatchGroundIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, -output);
    }

    public void setHatchIntakeOutput(double output){
        intakeMotors[1].set(ControlMode.PercentOutput, output);
    }

    public void updateIntakeIndicator() {
        for(int i = 0; i < intakeIndicator.length; i++)
            intakeIndicator[i] = false;
        intakeIndicator[intakeState] = true;
    }

    public void updateCargoIntakeState() {
        if(RobotContainer.rightButtons[2].get()) {

        } else if(bannerIR.get() && !isBannerTripped) {
            setCargoIntakeOutput(Constants.Setpoints.CARGO_HOLD_SPEED);
        } else if(isBannerTripped) {
            setCargoIntakeOutput(0);
            isBannerTripped = false;
        }
    }
    public void updateOuttakeState() {
            outtakeState = intakeState;
    }

    @Override
    public void initDefaultCommand() {
    }
}
