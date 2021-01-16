/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

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
  String trajectoryJSON;
  Trajectory trajectory;

  public PathWeaverTest(DrivetrainSubsystem drivetrainSubsystem) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      addRequirements(drivetrainSubsystem);
      trajectoryJSON = "paths/Test.wpilib.json";
      trajectory = new Trajectory();

      List<String> trajectoryList = new ArrayList<>();
        try {
            //Class<?> cls = Class.forName("JSONparsing.class");
            //ClassLoader classLoader = cls.getClassLoader();
            //JsonReader reader = new JsonReader(new FileReader(classLoader.getResource("links.json").getFile()));
            JsonReader reader = new JsonReader(new FileReader(""));
            reader.beginObject();
            while (reader.hasNext()) {
                String value = reader.nextString();
                trajectoryList.add(value);
            }
            reader.endObject();
            reader.close();
        } catch (FileNotFoundException fnfe) {
            fnfe.printStackTrace();
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        System.out.println(trajectoryList.toString());
    }


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
