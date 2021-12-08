package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;

public class ZeroWristEncoder extends InstantCommand {
    public ZeroWristEncoder() {
        requires(RobotContainer.wrist);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        RobotContainer.wrist.setEncoderPosition(Wrist.upperLimitEncoderCounts);
        RobotContainer.wrist.setAbsolutePosition(RobotContainer.wrist.getAngle());
        double calibrationValue = -RobotContainer.wrist.getPosition();
        // RobotContainer.controls.writeIniFile("Wrist", "Encoder_Calibration", String.valueOf(calibrationValue));
    }
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
