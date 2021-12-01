/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.constants.Constants;

public class IntakeExtend extends Subsystem {

    DoubleSolenoid harpoonExtend = new DoubleSolenoid(Constants.CAN.pcmOne, Constants.SolenoidAddresses.hatchIntakeExtendForward, Constants.SolenoidAddresses.hatchIntakeExtendReverse);

    public IntakeExtend() {
        super("IntakeExtend");

    }

    public boolean getHarpoonExtendStatus(){
        return harpoonExtend.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void setHarpoonExtend(boolean state){
        harpoonExtend.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void updateShuffleboard() {
    }

    public void updateSmartDashboard() {
    }

    @Override
    public void initDefaultCommand() {
    }
}
