// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.autoroutines;

// import java.util.List;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class GetAutonomousCommand extends CommandBase {
//   /**
//    * Creates a new GetAutonomousCommand.
//    */
//     /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   DrivetrainSubsystem drivetrainSubsystem;
//   public Command getAutonomousCommand(DrivetrainSubsystem drivetrainSubsystem) {
//     this.drivetrainSubsystem = drivetrainSubsystem;

//     // Create a voltage constraint to ensure we don't accelerate too fast
//     var autoVoltageConstraint =
//         new DifferentialDriveVoltageConstraint(
//             new SimpleMotorFeedforward(Constants.ksVolts,
//                                        Constants.kvVoltSecondsPerMeter,
//                                        Constants.kaVoltSecondsSquaredPerMeter),
//             Constants.kDriveKinematics,
//             10);

//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
//                              Constants.kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(Constants.kDriveKinematics)
//             // Apply the voltage constraint
//             .addConstraint(autoVoltageConstraint);

//     // An example trajectory to follow.  All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(
//             new Translation2d(1, 1),
//             new Translation2d(2, -1)
//         ),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         // Pass config
//         config
//     );

//     RamseteCommand ramseteCommand = new RamseteCommand(
//         exampleTrajectory,
//         drivetrainSubsystem::getPose,
//         new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
//         new SimpleMotorFeedforward(Constants.ksVolts,
//                                    Constants.kvVoltSecondsPerMeter,
//                                    Constants.kaVoltSecondsSquaredPerMeter),
//         Constants.kDriveKinematics,
//         drivetrainSubsystem::getWheelSpeeds,
//         new PIDController(Constants.kPDriveVel, 0, 0),
//         new PIDController(Constants.kPDriveVel, 0, 0),
//         // RamseteCommand passes volts to the callback
//         drivetrainSubsystem::tankDriveVolts,
//         DrivetrainSubsystem.drive
//     );

//     // Reset odometry to the starting pose of the trajectory.
//     drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return ramseteCommand.andThen(() -> DrivetrainSubsystem.drive.tankDriveVolts(0, 0));
//   }

// }
