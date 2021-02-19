// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAnglePIDCommand extends PIDCommand {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private double gyroGoal;

  /** Creates a new TurnToAnglePID. */
  public TurnToAnglePIDCommand(double gyroGoal, DrivetrainSubsystem drivetrainSubsystem) {
    
    super(
        // The controller that the command will use
        new PIDController(1.1, 0, 0.2),
        // This should return the measurement
        drivetrainSubsystem::getAngle,
        // This should return the setpoint (can also be a constant)
        gyroGoal,
        // This uses the output
        output -> {
        if(gyroGoal < 0){
          drivetrainSubsystem.drive(-output, output);
          System.out.println("Motor output: " + output);
        }
        else {
          drivetrainSubsystem.drive(output, -output);
          System.out.println("Motor output: " + output);
        }
      }
    );
        // Use the output h;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gyroGoal = gyroGoal;
    drivetrainSubsystem.resetGyro();
    getController().setTolerance(3.0);

  }

  @Override
    public void initialize() {
      super.initialize();
      drivetrainSubsystem.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return getController().atSetpoint();
  }
}
