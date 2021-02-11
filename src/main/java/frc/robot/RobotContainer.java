/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.shooting.TestShootingCommand;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.PathWeaverTest;
import frc.robot.commands.drivetrain.ResetEncodersCommand;
import frc.robot.commands.drivetrain.SlalomTestCommandGroup;
import frc.robot.commands.shifting.ToggleShiftingCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShiftGearsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    public final Joystick driverStationJoy;
    public DrivetrainSubsystem drivetrainSubsystem;
    public ShiftGearsSubsystem shiftGearsSubsystem;
    public ShooterSubsystem shooterSubsystem;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        driverStationJoy = new Joystick(Constants.DRIVER_STATION_JOY);

        drivetrainSubsystem = new DrivetrainSubsystem();
        shiftGearsSubsystem = new ShiftGearsSubsystem();
        shooterSubsystem = new ShooterSubsystem();

        drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.drive(getLeftY(), getRightY()), drivetrainSubsystem));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings()
    {
        setJoystickButtonWhenPressed(driverStationJoy, 1, new SlalomTestCommandGroup(drivetrainSubsystem));
        setJoystickButtonWhenPressed(driverStationJoy, 5, new ResetEncodersCommand(drivetrainSubsystem));
        setJoystickButtonWhenPressed(driverStationJoy, 2, new PathWeaverTest(drivetrainSubsystem));

        setJoystickButtonWhenPressed(driverStationJoy, 7, new DriveForDistanceCommand(drivetrainSubsystem, 36, 1, 1));
        //Shift Gears
        setJoystickButtonWhenPressed(driverStationJoy, 11, new ToggleShiftingCommand(shiftGearsSubsystem));

        //testing shooter
        setJoystickButtonWhileHeld(driverStationJoy, 8, new TestShootingCommand(shooterSubsystem, 1.0));
        setJoystickButtonWhileHeld(driverStationJoy, 9, new TestShootingCommand(shooterSubsystem, -1.0));

    }

    public double getLeftY() {
        if(driverStationJoy.getRawAxis(1) >= .1 || driverStationJoy.getRawAxis(1) <= -.1){
            return driverStationJoy.getRawAxis(1);
        }else{
            return 0;
        }
    // return leftJoy.getRawAxis(RobotMap.DRIVERSTATION_LEFT_Y_AXIS);
    }

  // Gets the Y direction of the left drive joystick
  public double getLeftX() {
    return driverStationJoy.getX();
  }

  // Gets the Y direction of the right drive joystick
  public double getRightY() {
    if(driverStationJoy.getRawAxis(3) >= .1 || driverStationJoy.getRawAxis(3) <= -.1){
      return driverStationJoy.getRawAxis(3);
    }else{
      return 0;
    }
    // return rightJoy.getRawAxis(RobotMap.DRIVERSTATION_RIGHT_Y_AXIS);
  }

  // Gets the X direction of the right drive joystick
  public double getRightX() {
    return driverStationJoy.getX();
  }
  
  private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whileHeld(command);
  }

  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }

  public Command getAutonomousCommand() {
    System.out.println("inside of robot container wee woo");
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrainSubsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        drivetrainSubsystem::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrainSubsystem::tankDriveVolts,
        drivetrainSubsystem
    );

    // Reset odometry to the starting pose of the trajectory.
    drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
  }

  
}
