// package frc.robot.commands.autonomous.routines;

/*
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.List;

public class Toast_Auto extends SequentialCommandGroup {
    public Toast_Auto(DriveTrain driveTrain) {
        int[][] waypointsRaw = {
            {0,0,0},
            {4,0,0},
            {0,0,0}
            
         };
         Pose2d[] waypoints = new Pose2d[waypointsRaw.length];
         for (int j = 0; j < waypointsRaw.length; j++) {
             waypoints[j] = new Pose2d(Units.inchesToMeters(waypointsRaw[j][0]), Units.inchesToMeters(waypointsRaw[j][1]), new Rotation2d(Units.degreesToRadians(waypointsRaw[j][2])));
         }
        
         Pose2d startPosition = waypoints[0];

    

         for(int i = 0; i < waypoints.length - 1; i++) {
                 if (i != 0) {
                         configA.setEndVelocity(configA.getMaxVelocity());
                         configA.setStartVelocity(configA.getMaxVelocity());
                 }
                 if (i == waypoints.length - 2) {
                         configA.setEndVelocity(0);
                 }
                 Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                 List.of(),
                 waypoints[i + 1],
                configA);


             var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
             addCommands(command);
         }
    }
}

*/
