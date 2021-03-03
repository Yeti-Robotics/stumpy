// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnForAngleCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchLayoutBRed extends SequentialCommandGroup {
/** Creates a new GalacticSearchLayoutBRed. */
  public GalacticSearchLayoutBRed(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(
      new ToggleIntakeCommand(intakeSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 60, 0.7, 0.7),
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      new TurnToAnglePIDCommand(56.31, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 108.166, 0.7, 0.7),
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      new TurnToAnglePIDCommand(-112.62, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 108.166, 0.7, 0.7),
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      new TurnToAnglePIDCommand(56.31, drivetrainSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, 1, 1)
    );
    
  }
}
