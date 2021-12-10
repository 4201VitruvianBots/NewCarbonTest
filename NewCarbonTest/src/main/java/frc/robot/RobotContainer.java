// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.ManualWristUp;
import frc.robot.commands.SetArcadeDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_driveTrain = new DriveTrain();
  public final static Intake intake = new Intake();
  public static Wrist wrist = new Wrist();
  public static DoubleSupplier m_throttle;
  public static DoubleSupplier m_turn;
  static final Joystick leftJoystick = new Joystick(Constants.leftJoystick);
  static final Joystick rightJoystick = new Joystick(Constants.rightJoystick);
  static final Joystick xBoxController = new Joystick(Constants.xBoxController);
  
  static Joystick testContoller = new Joystick(4);
  private static boolean init = false;
  
  public final Button[] leftButtons = new Button[2];
  public final static Button[] rightButtons = new Button[2];
  public final Button[] xBoxButtons = new Button[10];
  public final Button[] xBoxPOVButtons = new Button[8];
  
  public Button xBoxLeftTrigger, xBoxRightTrigger;
  public Button[] testButtons = new Button[10];
  
  // private final m_command = new ExampleCommand(m_exampleSubsystem);
  //
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain, () -> leftJoystick.getRawAxis(1), () -> rightJoystick.getRawAxis(0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xBoxButtons[0].whenPressed(new ManualWristUp(wrist));
  }
  
  private void initializeSubsystems() {
  
    if(RobotBase.isReal()) {
      m_driveTrain.setDefaultCommand(
       new SetArcadeDrive(m_driveTrain,
       () -> leftJoystick.getRawAxis(1), 
       () -> rightJoystick.getRawAxis(0)));
      
    }
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
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
