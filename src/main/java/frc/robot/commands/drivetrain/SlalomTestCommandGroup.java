package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlalomTestCommandGroup extends SequentialCommandGroup {
    public SlalomTestCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
        super();
        addCommands(
            // brute force of the slalom course
                new DriveForDistanceCommand(drivetrainSubsystem, 15, 0.5, 0.5),
                new TurnToAnglePIDCommand(50, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 56, 0.5, 0.5),
                new TurnToAnglePIDCommand(-40, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 90, 0.5, 0.5),
                new TurnToAnglePIDCommand(-60, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 38, 0.5, 0.5),

                new TurnToAnglePIDCommand(80, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 28, 0.5, 0.5),
                new TurnToAnglePIDCommand(69, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 23, 0.5, 0.5),
                new TurnToAnglePIDCommand(70, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 34, .5, .5),
                new TurnToAnglePIDCommand(85, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 54, .5, .5 ),
                new TurnToAnglePIDCommand(-90, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 92, .5, .5),
                new TurnToAnglePIDCommand(-45, drivetrainSubsystem)
        );
    }
}