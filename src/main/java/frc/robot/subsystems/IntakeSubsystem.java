// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public static TalonSRX intakeRoller;
  public static IntakeStatus intakeStatus;
  public static DoubleSolenoid pistons;

  public enum IntakeStatus{
    DOWN, UP
}
  public IntakeSubsystem() {
    intakeRoller = new TalonSRX(Constants.INTAKE_ROLLER_TALON);
    pistons = new DoubleSolenoid(Constants.INTAKE_PISTONS_SOLENOID[0], Constants.INTAKE_PISTONS_SOLENOID[1]);

  }

  public void extend(){
    pistons.set(DoubleSolenoid.Value.kForward);
    intakeStatus = IntakeStatus.DOWN;
  }

  public void retract(){
    pistons.set(DoubleSolenoid.Value.kReverse);
    intakeStatus = IntakeStatus.UP;
  }

  public void intakeIn() {
    intakeRoller.set(ControlMode.PercentOutput, Constants.INTAKE_IN_SPEED);
  }

  public void intakeOut() {
    intakeRoller.set(ControlMode.PercentOutput, Constants.INTAKE_OUT_SPEED);
  }

  public void stopIntake() {
    intakeRoller.set(ControlMode.PercentOutput, 0);
  }

  public static IntakeStatus getIntakePosition(){
    return intakeStatus;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
