/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.ResetEncodersCommand;
import frc.robot.commands.drivetrain.SlalomTestCommandGroup;
import frc.robot.commands.shifting.ToggleShiftingCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShiftGearsSubsystem;

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

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        driverStationJoy = new Joystick(Constants.DRIVER_STATION_JOY);

        drivetrainSubsystem = new DrivetrainSubsystem();
        shiftGearsSubsystem = new ShiftGearsSubsystem();

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
        setJoystickButtonWhileHeld(driverStationJoy, 1, new SlalomTestCommandGroup(drivetrainSubsystem));
        setJoystickButtonWhenPressed(driverStationJoy, 5, new ResetEncodersCommand(drivetrainSubsystem));
        //Shift Gears
        setJoystickButtonWhenPressed(driverStationJoy, 11, new ToggleShiftingCommand(shiftGearsSubsystem));
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
}
