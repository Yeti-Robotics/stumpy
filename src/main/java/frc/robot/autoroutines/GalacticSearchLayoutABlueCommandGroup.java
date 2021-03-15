// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnToAnglePIDCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchLayoutABlueCommandGroup extends SequentialCommandGroup {
  /** Creates a new GalacticSearchLayoutABlue. */
  public GalacticSearchLayoutABlueCommandGroup(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(
      //starting aligned with the first ball; move forward 150 in
      new DriveForDistanceCommand(drivetrainSubsystem, 150.0, 0.5, 0.5), 
      //intake ball
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      //turn 71.565° CCW
      new TurnToAnglePIDCommand(-71.565, drivetrainSubsystem),
      //move forward 30 sqrt(10) or 94.86833 inches
      new DriveForDistanceCommand(drivetrainSubsystem, 30 * Math.sqrt(10.0), 0.5, 0.5),
      //intake ball
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      //turn back 71.565° + 26.565° CW
      new TurnToAnglePIDCommand(71.565 + 26.565, drivetrainSubsystem),
      //move forward 30 sqrt(5) or 67.08204 inches
      new DriveForDistanceCommand(drivetrainSubsystem, 30 * Math.sqrt(5.0), 0.5, 0.5),
      //intake ball
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      //turn back 26.565° CCW
      new TurnToAnglePIDCommand(-26.565, drivetrainSubsystem),
      //move 60 inches to end zone
      new DriveForDistanceCommand(drivetrainSubsystem, 60.0, 0.5, 0.5)
    );
  }
}
