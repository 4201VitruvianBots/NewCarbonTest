package frc.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // USB PORTS

    public static int cargoIntakeMotor = 60;
    public static int hatchIntakeMotor = 61;


    public static int wristMotor = 40;
    // DIOs
    public static int bannerIR = 0;
    public static int hatchSensor = 1;
    public static int elevatorBottom = 2;
    public static int elevatorTop = 3;
    public static int elevatorMid = 4;
    public static int wristBottom = 5;
    public static int wristTop = 6;
    public static int robotSwitch = 9;
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;

    // CAN ADDRESSES
    public static int pdp = 0;
    public static int PCMOne = 11;

    public static int leftFrontDriveMotor = 20;
    public static int leftRearDriveMotor = 21;
    public static int rightFrontDriveMotor = 22;
    public static int rightRearDriveMotor = 23;
    public static int climbDriveMotor = 24;

    //Solenoid addresses ¿holá señor?
    public static int driveTrainShifterForward = 0;
    public static int driveTrainShifterReverse = 1;

    public static final class DriveConstants {
        public static final int[] kLeftEncoderPorts = new int[]{10, 11};
        public static final int[] kRightEncoderPorts = new int[]{12, 13};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = Units.inchesToMeters(21.5);
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearingLow = 5.0;
        public static final double kDriveGearingHigh = 5.0;


        public static final int kMagEncoderCPR = 4096;
        public static final int kFalconEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
        public static final double kEncoderDistancePerPulseLow =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / (double) kFalconEncoderCPR * kDriveGearingLow;
        public static final double kEncoderDistancePerPulseHigh =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / (double) kFalconEncoderCPR * kDriveGearingLow;
        public static final double kEncoderDistancePerPulseSim =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kMagEncoderCPR * kDriveGearingHigh;

        public static final boolean kGyroReversed = true;

        public static final boolean inSlowGear = true; // True = 1 : 14.14, False = 1 : 7.49

        public static final double ksVolts = 0.75514;//3/21/21//inSlowGear ? 0.683 : 0.81;
        public static final double kvVoltSecondsPerMeter = 2.1851;//3/21/21//3.21inSlowGear ? 3.19 : 1.74;
        public static final double kaVoltSecondsSquaredPerMeter = 0.57574;//3/21/21//inSlowGear ? 0.227 : 0.301;

        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 3.34;//inSlowGear ? 3.41 : 2.08; // originally 1.5
        public static final double kaVoltSecondsSquaredPerRadian = 0.19;//inSlowGear ? 0.111 : -0.0132; // originally 0.3

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
    }


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