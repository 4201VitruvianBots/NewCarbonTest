// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
// private final DriveTrain = m_driveTrain;

public class RobotContainer {

  private final PowerDistributionPanel pdp = new PowerDistributionPanel();
  private final DriveTrain m_driveTrain = new DriveTrain();

  static final Joystick leftJoystick = new Joystick(Constants.leftJoystick);
 static final Joystick rightJoystick = new Joystick(Constants.rightJoystick);
 private static boolean init = false;

 

  // The robot's subsystems and commands are defined here...
  private enum CommandSelector { 
    TOAST_AUTO

  }
  // private CommandSelector selectCommand() {
  //   return CommandSelector.values()[m_autoChooser.getSelected()];}

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
   
    
      // m_autoChooser.addDefault("Toast_Auto", CommandSelector.TOAST_AUTO.ordinal());
      // for(Enum commandEnum : CommandSelector.values())
      //     if(commandEnum != CommandSelector.TOAST_AUTO)
      //         m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());

      // SmartDashboard.putData(m_autoChooser);
  }
    
  // public void autonomousInit() {
  //   if (RobotBase.isReal()) {
  //       m_driveTrain.resetEncoderCounts();
  //       m_driveTrain.resetOdometry(m_driveTrain.getRobotPose(), m_FieldSim.getRobotPose().getRotation());
  //   } else {
  //       m_FieldSim.initSim();
  //       m_driveTrain.resetEncoderCounts();
  //       m_driveTrain.resetOdometry(m_FieldSim.getRobotPose(), m_FieldSim.getRobotPose().getRotation());
  //   }


    
    // Configure the button bindings
      

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
public static boolean getInitializationState() {

  return init;
}

public static void setInitializationState(boolean state){
  init = state;

}


  private void initializeSubsystems() {
   

    if(RobotBase.isReal()) {
      m_driveTrain.setDefaultCommand(
      new SetArcadeDrive(m_driveTrain,
      () -> leftJoystick.getRawAxis(1), 
      () -> rightJoystick.getRawAxis(0)));
      
    }
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
