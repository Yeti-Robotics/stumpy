// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeckSubsystem extends SubsystemBase {
  /** Creates a new NeckSubsystem. */
  private VictorSPX neckRoller; //pinch roller

  public NeckSubsystem() {
    neckRoller = new VictorSPX(Constants.NECK_ROLLER_VICTOR);
  }

  public void pinchIn(){
    neckRoller.set(ControlMode.PercentOutput, Constants.NECK_IN_SPEED);
}

  public void pinchOut(){
    neckRoller.set(ControlMode.PercentOutput, Constants.NECK_OUT_SPEED);
}

  public void pinchStop(){
    neckRoller.set(ControlMode.PercentOutput, 0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
