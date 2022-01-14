/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.hopper.HopperInCommand;
import frc.robot.commands.hopper.HopperOutCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.neck.NeckInCommand;
import frc.robot.commands.neck.NeckOutCommand;
import frc.robot.commands.shooting.TestShootingCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.drivetrain.BarrelRacingCommandGroup;
import frc.robot.commands.drivetrain.BouncePathCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.DriveForDistancePIDCommand;
import frc.robot.commands.drivetrain.PathWeaverTest;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.drivetrain.ResetEncodersCommand;
import frc.robot.commands.drivetrain.SlalomTestCommandGroup;
import frc.robot.commands.shifting.ToggleShiftingCommand;
import frc.robot.subsystems.*;

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
    public IntakeSubsystem intakeSubsystem;
    public HopperSubsystem hopperSubsystem;
    public NeckSubsystem neckSubsystem;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        driverStationJoy = new Joystick(Constants.DRIVER_STATION_JOY);

        drivetrainSubsystem = new DrivetrainSubsystem();
        shiftGearsSubsystem = new ShiftGearsSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        hopperSubsystem = new HopperSubsystem();
        neckSubsystem = new NeckSubsystem();

        switch (drivetrainSubsystem.getDriveMode()) {
          case TANK:
          drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()), drivetrainSubsystem));
          break;
          case CHEEZY:
          drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()), drivetrainSubsystem));
          break;
          case ARCADE:
          drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.arcadeDrive(getLeftY(), getRightX()), drivetrainSubsystem));
      }


        //driving lol
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
        //Shift Gears
        setJoystickButtonWhenPressed(driverStationJoy, 11, new ToggleShiftingCommand(shiftGearsSubsystem));
        
        setJoystickButtonWhileHeld(driverStationJoy, 1, new TestShootingCommand(shooterSubsystem, 0.3) );
        //full shooter system test
        // setJoystickButtonWhenPressed(driverStationJoy, 5, new ToggleIntakeCommand(intakeSubsystem));

        // setJoystickButtonWhileHeld(driverStationJoy, 1, new IntakeInCommand(intakeSubsystem));
        // setJoystickButtonWhileHeld(driverStationJoy, 6, new IntakeOutCommand(intakeSubsystem));

        // setJoystickButtonWhileHeld(driverStationJoy, 2, new HopperInCommand(hopperSubsystem));
        // setJoystickButtonWhileHeld(driverStationJoy, 7, new HopperOutCommand(hopperSubsystem));

        // setJoystickButtonWhileHeld(driverStationJoy, 3, new NeckInCommand(neckSubsystem));
        // setJoystickButtonWhileHeld(driverStationJoy, 8, new NeckOutCommand(neckSubsystem));

        // setJoystickButtonWhileHeld(driverStationJoy, 4, new TestShootingCommand(shooterSubsystem, 1.0));
        // setJoystickButtonWhileHeld(driverStationJoy, 9, new TestShootingCommand(shooterSubsystem, -1.0));
        //setJoystickButtonWhenPressed(driverStationJoy, 8, new BarrelRacingCommandGroup(drivetrainSubsystem));
        //setJoystickButtonWhenPressed(driverStationJoy, 9, new SlalomPathCommandGroup(drivetrainSubsystem));
        // setJoystickButtonWhenPressed(driverStationJoy, 9, new DriveForDistancePIDCommand(drivetrainSubsystem, 60));
    

    }

    public double getLeftY() {
        if(driverStationJoy.getRawAxis(1) >= .1 || driverStationJoy.getRawAxis(1) <= -.1){
            return driverStationJoy.getRawAxis(0);
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
      return driverStationJoy.getRawAxis(2);
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

  // public Command getAutonomousCommand() {}

  
}
