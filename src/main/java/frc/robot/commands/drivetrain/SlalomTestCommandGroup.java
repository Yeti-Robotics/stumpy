package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlalomTestCommandGroup extends SequentialCommandGroup {
    public SlalomTestCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
        super();
        addCommands(
                new DriveForDistanceCommand(drivetrainSubsystem, 72, 1.0, 1.0),
                new TurnForAngleCommand(drivetrainSubsystem, 45, 0.5, 0.5),
                new DriveForDistanceCommand(drivetrainSubsystem, 120, 1.0, 1.0)
        );
    }
}