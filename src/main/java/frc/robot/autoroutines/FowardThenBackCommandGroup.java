package frc.robot.autoroutines;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.subsystems.*;

public class FowardThenBackCommandGroup extends SequentialCommandGroup {
    public FowardThenBackCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
       super();
        addCommands(
                new DriveForDistanceCommand(drivetrainSubsystem, 62, .5,  .5 ),
                new DriveForDistanceCommand(drivetrainSubsystem, -62, .5, .5)
        );
    }
}