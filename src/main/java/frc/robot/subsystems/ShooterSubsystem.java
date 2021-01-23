/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private TalonFX rightShooter;
  private TalonFX leftShooter;
  public ShooterSubsystem() {
    rightShooter = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR);
    leftShooter = new TalonFX(Constants.LEFT_SHOOTER_MOTOR);

    rightShooter.setNeutralMode(NeutralMode.Brake);
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setInverted(true);
    leftShooter.setInverted(false);
  }

  public void spin(double power){
    leftShooter.set(ControlMode.PercentOutput, power);
    rightShooter.set(ControlMode.PercentOutput, power);
}

public void stopSpin(){
    leftShooter.set(ControlMode.PercentOutput, 0);
    rightShooter.set(ControlMode.PercentOutput, 0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
