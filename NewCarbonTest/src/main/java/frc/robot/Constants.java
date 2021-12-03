// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // USB PORTS
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
	public static int driveTrainShiftersForward = 0; 
    public static int driveTrainShiftersReverse = 1; 
    public static int pcm = 11;
    public static int leftFrontDriveMotor = 20;
    public static int leftRearDriveMotor = 21; 
    public static int rightFrontDriveMotor = 22;
    public static int rightRearDriveMotor = 23;
    public static int wristMotor = 40;
    public static int climbMotor = 50;
    public static int cargoIntakeMotor = 60;
    public static int hatchIntakeMotor = 61;
    
    // DIOs
    public static int bannerIR = 0;
    public static int hatchSensor = 1;
    public static int elevatorBottom = 2;
    public static int elevatorTop = 3;
    public static int elevatorMid = 4;
    public static int wristBottom = 5;
    public static int wristTop = 6;
    public static int robotSwitch = 9;

    //Cargo/Hatch Speed Values
    public static double CARGO_INTAKE_SPEED = -0.7;
    public static double CARGO_OUTTAKE_SPEED = 0.7;
    public static double CARGO_HOLD_SPEED = -0.2;
    public static double HATCH_INTAKE_SPEED = -0.8;
    public static double HATCH_HOLD_SPEED = 0;
    public static double HATCH_OUTTAKE_SPEED = 0.8;
    public static double HATCH_GROUND_INTAKE_SPEED = -0.8;
    public static double HATCH_GROUND_HOLD_SPEED = -0.1;
    public static double HATCH_GROUND_OUTTAKE_SPEED = 1;
}
