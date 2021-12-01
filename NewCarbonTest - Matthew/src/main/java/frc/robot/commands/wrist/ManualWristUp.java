package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;

public class ManualWristUp extends Command {
    public ManualWristUp() {
        // Use requires() here to declare subsystem dependencies
        requires(RobotContainer.wrist);
        setTimeout(5);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        RobotContainer.wrist.setDirectOutput(0.4);
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return isTimedOut() || RobotContainer.wrist.getLimitSwitchState(1);
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        if(RobotContainer.wrist.getLimitSwitchState(1)) {
            Wrist.controlMode = 1;
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
