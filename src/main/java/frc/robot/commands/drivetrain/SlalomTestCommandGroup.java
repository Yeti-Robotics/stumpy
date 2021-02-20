package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlalomTestCommandGroup extends SequentialCommandGroup {
    public SlalomTestCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
        super();
        addCommands(
            // brute force of the slalom course
                new DriveForDistanceCommand(drivetrainSubsystem, 8, 0.5, 0.5),
                new TurnToAnglePIDCommand(45, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 84, 0.5, 0.5),
                new TurnToAnglePIDCommand(-45, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 64, 0.5, 0.5),
                new TurnToAnglePIDCommand(-45, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 48, 0.5, 0.5),
                new TurnToAnglePIDCommand(90, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 24, 0.5, 0.5)

                // new TurnToAnglePIDCommand(90, drivetrainSubsystem),
                // new DriveForDistanceCommand(drivetrainSubsystem, 84, 0.5, 0.5),
                // new TurnToAnglePIDCommand(90, drivetrainSubsystem)
        );
    }
}