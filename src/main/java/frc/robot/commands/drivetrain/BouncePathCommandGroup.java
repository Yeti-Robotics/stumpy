// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnForAngleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BouncePathCommandGroup extends SequentialCommandGroup {
  /** Creates a new BounchPathCommandGroup. */
  public BouncePathCommandGroup(DrivetrainSubsystem drivetrainSubsystem, double power) {
    addCommands(
      new DriveForDistanceCommand(drivetrainSubsystem, 30, 0.5, 0.5),
      new TurnToAnglePIDCommand(-26.57, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 30, 0.5, 0.5),
      new TurnToAnglePIDCommand(26.57, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 60, 0.5, 0.5),
      new TurnToAnglePIDCommand(63.43, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem,60, 0.5, 0.5),
      new TurnToAnglePIDCommand(-63.43, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem,30, 0.5, 0.5),
      new TurnToAnglePIDCommand(-14.04, drivetrainSubsystem), 
      new DriveForDistanceCommand(drivetrainSubsystem,30, 0.5, 0.5),
      new TurnToAnglePIDCommand(14.04, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem,30, .5, .5),
      new TurnToAnglePIDCommand(75.96, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem,30, .5, .5 ),
      new TurnToAnglePIDCommand(-76.96, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem,30, .5, .5),
      new TurnToAnglePIDCommand(-14.04, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 30, .5, .5)
    );
  }
}
