package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class SetArcadeDrive extends CommandBase {

    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_throttle, m_turn;



    public SetArcadeDrive(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn){
        m_turn = turn;
        m_throttle = throttle;
        m_driveTrain = driveTrain;
        

        addRequirements(driveTrain);
    }
    @Override
    public void initialize() {
    
}

    @Override
    public void execute() {
        double joystickY = (Math.abs(m_throttle.getAsDouble()) > 0.05) ? m_throttle.getAsDouble() : 0;
        double joystickX = (Math.abs(m_turn.getAsDouble()) > 0.05) ? m_turn.getAsDouble() : 0;

        double throttle = joystickY; //Y is throttle because when you push the joystick up, it will go forward.
        double turn = joystickX;
        
       throttle = throttle < 0 ? Math.max(-0.7, throttle) : throttle;
       
    
        m_driveTrain.setMotorArcadeDrive(throttle, turn);
   
    }  



    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}