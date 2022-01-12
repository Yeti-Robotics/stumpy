/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private WPI_TalonFX rightShooter;
  private WPI_TalonFX leftShooter;
  
  public ShooterSubsystem() {
  
    rightShooter = new WPI_TalonFX(Constants.RIGHT_SHOOTER_MOTOR);
    leftShooter = new WPI_TalonFX(Constants.LEFT_SHOOTER_MOTOR);
    
    rightShooter.setNeutralMode(NeutralMode.Brake);
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setInverted(false);
    leftShooter.setInverted(true);

    rightShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
    leftShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
  }

  
  public void spin(double power){
    leftShooter.set(ControlMode.PercentOutput, power);
    rightShooter.set(ControlMode.PercentOutput, power);
  }

public void stopSpin(){
    leftShooter.set(ControlMode.PercentOutput, 0);
    rightShooter.set(ControlMode.PercentOutput, 0);
}

  public double getAverageEncoder() {
    return ((leftShooter.getSelectedSensorVelocity() + rightShooter.getSelectedSensorVelocity()) /2.0) * (4.0/3.0) * (600.0/4096.0); // gear ratio of 4:3
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("average shooter encoder: " + getAverageEncoder());
  }
}
