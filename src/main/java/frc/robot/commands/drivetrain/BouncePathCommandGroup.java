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
      new DriveForDistanceCommand(drivetrainSubsystem, 2.5*12, power, power),
      new TurnToAnglePIDCommand(90, drivetrainSubsystem), //ccw
      new DriveForDistanceCommand(drivetrainSubsystem, 5*12, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, -5, power, power),
      new TurnToAnglePIDCommand(30, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, -8*12, power, power)
      // new TurnForAngleCommand(drivetrainSubsystem, -90, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, 2.5, power, power),
      // new TurnForAngleCommand(drivetrainSubsystem, -45, power, power), //angle opp
      // new DriveForDistanceCommand(drivetrainSubsystem, 7.5, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, -7.5, power, power),
      // new TurnForAngleCommand(drivetrainSubsystem, 135, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, 4, power, power),
      // new TurnForAngleCommand(drivetrainSubsystem, -90, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, 4, power, power),
      // new TurnForAngleCommand(drivetrainSubsystem, -45, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, 7.5, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, -5, power, power),
      // new TurnForAngleCommand(drivetrainSubsystem, 90, power, power),
      // new DriveForDistanceCommand(drivetrainSubsystem, 5, power, power)
    );
  }
}
