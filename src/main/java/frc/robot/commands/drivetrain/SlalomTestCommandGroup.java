package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlalomTestCommandGroup extends SequentialCommandGroup {
    public SlalomTestCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
        super();
        addCommands(
            // brute force of the slalom course
                new DriveForDistanceCommand(drivetrainSubsystem, 72, 0.5, 0.5),
                new TurnForAngleCommand(drivetrainSubsystem, 35, 0.5, 0.5),
                new DriveForDistanceCommand(drivetrainSubsystem, 120, 0.5, 0.5),
                new TurnForAngleCommand(drivetrainSubsystem, 15, 0.5, 0.5),
                new DriveForDistanceCommand(drivetrainSubsystem, 45, 0.25, 0.25),
                new TurnForAngleCommand(drivetrainSubsystem, -90, 0.5, 0.5),
                new DriveForDistanceCommand(drivetrainSubsystem, 50, 0.25, 0.25),
                new TurnForAngleCommand(drivetrainSubsystem, -90, 0.5, 0.5),
                new DriveForDistanceCommand(drivetrainSubsystem, 50, 0.25, 0.25),
                new TurnForAngleCommand(drivetrainSubsystem, -90, 0.5, 0.5)
        );
    }
}