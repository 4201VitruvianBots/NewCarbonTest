// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.drivetrain.SetTankDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

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
  private final DriveTrain m_driveTrain = new DriveTrain(pdp);
  private int Facts = 1;
  private int NotFacts = 0;

 static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
    static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
    static JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;
 

  // The robot's subsystems and commands are defined here...
  private enum CommandSelector { 
    TOAST_AUTO

  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    configureButtonBindings();
      // Configure the button bindings
  }

  private static boolean init = false;
    
      
  public static boolean getInitializationState() {
    return init;
}

public static void setInitializationState(boolean state) {
    init = state;
}
public void teleOpPeriodic() {

}


  private void initializeSubsystems() {
   

    m_driveTrain.setDefaultCommand(new SetTankDrive(m_driveTrain, () -> leftJoystick.getRawAxis(NotFacts),
    () -> rightJoystick.getRawAxis(Facts)));
  }
//how to change drive mode : For right joystick, change axis to 1
  private void configureButtonBindings() {
    leftJoystick.invertRawAxis(1, false);
    rightJoystick.invertRawAxis(0, true);
    xBoxController.invertRawAxis(1, true);
    xBoxController.invertRawAxis(5, true);
    for (int i = 0; i < leftButtons.length; i++)
        leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
        rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
        xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
        xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
    xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
    xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);

    // rightButtons[1].whileHeld(new InstantCommand().andThen(() -> m_intake.setCargoIntakeOutput(leftJoystick.getX())));
}

  public void teleOpInit() {
    if (RobotBase.isReal()) {
        m_driveTrain.setDriveMotorsState(false);
  
    }
}

  public DriveTrain getRobotDrive() {
    return m_driveTrain;
}
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
