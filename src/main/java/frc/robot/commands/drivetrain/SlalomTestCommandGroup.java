package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlalomTestCommandGroup extends SequentialCommandGroup {
    public SlalomTestCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
        super();
        addCommands(
            // brute force of the slalom course
                new DriveForDistanceCommand(drivetrainSubsystem, 72/2, 0.5, 0.5),
                new TurnToAnglePIDCommand(35, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 84, 0.5, 0.5),
                // new TurnToAnglePIDCommand(-, drivetrainSubsystem),
                // new DriveForDistanceCommand(drivetrainSubsystem, 46/2, 0.5, 0.5),
                new TurnToAnglePIDCommand(-90, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 50/2, 0.5, 0.5),
                new TurnToAnglePIDCommand(90, drivetrainSubsystem),
                new DriveForDistanceCommand(drivetrainSubsystem, 50/2, 0.5, 0.5),
                new TurnToAnglePIDCommand(90, drivetrainSubsystem)
        );
    }
}