package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlalomTestCommandGroup extends SequentialCommandGroup {
    public SlalomTestCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
        super();
        addCommands(
                new DriveForDistanceCommand(drivetrainSubsystem, 72, 0.5, 0.5);
                new TurnForAngleCommand(drivetrainSubsystem, 45, 0.5, 0.5);
                new DriveForDistanceCommand(drivetrainSubsystem, 120, 0.5, 0.5);
        );
    }
}