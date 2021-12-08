// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetArcadeDrive;
import frc.robot.commands.SetTankDrive;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.wrist.ToggleWristState;
import frc.robot.commands.wrist.ZeroWristEncoder;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtend;
import frc.robot.subsystems.Wrist;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final PowerDistributionPanel pdp = new PowerDistributionPanel();
  private final DriveTrain m_driveTrain = new DriveTrain(pdp);
  public final static Intake intake = new Intake();
  public static Wrist wrist = new Wrist();
  public static IntakeExtend intakeExtend = new IntakeExtend();

  static final JoystickWrapper leftJoystick = new JoystickWrapper(Constants.USB.leftJoystick);
  static final JoystickWrapper rightJoystick = new JoystickWrapper(Constants.USB.rightJoystick);
  static final JoystickWrapper xBoxController = new JoystickWrapper(Constants.USB.xBoxController);
  
  static JoystickWrapper testContoller = new JoystickWrapper(4);
  private static boolean init = false;
  
  public final Button[] leftButtons = new Button[2];
  public final static Button[] rightButtons = new Button[2];
  public final Button[] xBoxButtons = new Button[4];
  public final Button[] xBoxPOVButtons = new Button[8];
  
  public Button xBoxLeftTrigger, xBoxRightTrigger;
  public Button[] testButtons = new Button[10];

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
  }

  public static boolean getInitializationState() {
    return init;
  }

  public static void setInitializationState(boolean state) {
    init = state;
  }

  //use tank drive
  // public void initializeSubsystems() {
  //   if(RobotBase.isReal()) {
  //     m_driveTrain.setDefaultCommand(
  //     new SetTankDrive(m_driveTrain,
  //             () -> -leftJoystick.getRawAxis(1),
  //             () -> rightJoystick.getRawAxis(1)));
  //   }
  // }

  //use arcade drive
  public void initializeSubsystems() {
    if(RobotBase.isReal()) {
      m_driveTrain.setDefaultCommand(
      new SetArcadeDrive(m_driveTrain,
              () -> -leftJoystick.getRawAxis(1),
              () -> rightJoystick.getRawAxis(1)));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    leftJoystick.invertRawAxis(1, true);
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

    //Intake Buttons
    xBoxButtons[4].whileHeld(new IntakeIntake());
    xBoxRightTrigger.whenPressed(new SetIntakeState(2));
    xBoxButtons[5].whenPressed(new SetIntakeState(0));

    //Wrist Buttons
    xBoxButtons[7].whenPressed(new ToggleWristState());
    xBoxPOVButtons[0].whenPressed(new ZeroWristEncoder());
  }

  public static double getXBoxRightY(){
    return -xBoxController.getRawAxis(5);
  }

  public static void enableXBoxRumbleTimed(double duration){
    Thread t = new Thread(() -> {
      setXBoxRumble(0.8);
      Timer.delay(duration);
      setXBoxRumble(0);
    });
    t.start();
  }

  private static void setXBoxRumble(double value) {
    xBoxController.setRumble(GenericHID.RumbleType.kLeftRumble, value);
    xBoxController.setRumble(GenericHID.RumbleType.kRightRumble, value);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void disabledInit() {
    setInitializationState(true);
  }

  public void robotPeriodic() {
  }

  public void teleOpInit() {
  }

  public void teleOpPeriodic() {
  }
}
