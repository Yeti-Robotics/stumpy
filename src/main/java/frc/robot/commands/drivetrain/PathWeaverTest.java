/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PathWeaverTest extends CommandBase {
  /**
   * Creates a new PathWeaverTest.
   */
  private final DrivetrainSubsystem drivetrainSubsystem;

  public PathWeaverTest(DrivetrainSubsystem drivetrainSubsystem) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      addRequirements(drivetrainSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PathWeaverTest initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("PathWeaverTest executing");
  
    String trajectoryJSON = "src/main/deploy/paths/Test.wpilib.json";
    Trajectory trajectory = new Trajectory(null);
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        System.out.println("inside try");
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        System.out.println("inside catch");
    }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PathWeaverTest ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("PathWeaverTest finished");
    return false;
  }
}
