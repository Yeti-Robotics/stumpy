// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//aiden's code, not the tuned one
package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlalomPathCommandGroup extends SequentialCommandGroup {
  /** Creates a new SlalomPathCommandGroup. */
  public SlalomPathCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistanceCommand(drivetrainSubsystem, 60, 0.5, 0.5),
      new TurnToAnglePIDCommand(45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 51.96152 , 0.5, 0.5),
      new TurnToAnglePIDCommand(-45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, 0.5, 0.5),
      new TurnToAnglePIDCommand(-45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 84.852813 , 0.5, 0.5),
      new TurnToAnglePIDCommand(78.690067, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 54.0833, 0.5, 0.5),
      new TurnToAnglePIDCommand(78.690067, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 54.0833, 0.5, 0.5),
      new TurnToAnglePIDCommand(45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 84.852813, .5, .5),
      new TurnToAnglePIDCommand(45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, .5, .5 ),
      new TurnToAnglePIDCommand(-45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 51.9615242, .5, .5),
      new TurnToAnglePIDCommand(45, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 60, .5, .5)
    );
  }
}
