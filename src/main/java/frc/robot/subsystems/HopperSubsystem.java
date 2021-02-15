// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  /** Creates a new HopperSubsystem. */
  private VictorSPX hopperRoller;

  public HopperSubsystem() {
    hopperRoller = new VictorSPX(Constants.HOPPER_ROLLER_VICTOR);

  }

  public void rollIn(){
    hopperRoller.set(ControlMode.PercentOutput, Constants.HOPPER_IN_SPEED);
}

  public void rollOut(){
    hopperRoller.set(ControlMode.PercentOutput, Constants.HOPPER_OUT_SPEED);
}

  public void stopHopper(){
    hopperRoller.set(ControlMode.PercentOutput, 0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
