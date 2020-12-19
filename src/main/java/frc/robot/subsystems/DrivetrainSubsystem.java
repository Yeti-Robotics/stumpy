package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private TalonFX rightFalcon1;
    private TalonFX rightFalcon2;
    private TalonFX leftFalcon1;
    private TalonFX leftFalcon2;

    public DrivetrainSubsystem() {
        rightFalcon1 = new TalonFX(Constants.RIGHT_FALCON_ONE);
        rightFalcon2 = new TalonFX(Constants.RIGHT_FALCON_TWO);
        leftFalcon1 = new TalonFX(Constants.LEFT_FALCON_ONE);
        leftFalcon2 = new TalonFX(Constants.LEFT_FALCON_TWO);

        rightFalcon1.setNeutralMode(NeutralMode.Brake);
        rightFalcon2.setNeutralMode(NeutralMode.Brake);
        leftFalcon1.setNeutralMode(NeutralMode.Brake);
        leftFalcon2.setNeutralMode(NeutralMode.Brake);

        leftFalcon1.setInverted(false);
        rightFalcon1.setInverted(true);
        rightFalcon2.setInverted(true);
        
        leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
        rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
    }

    public void drive(double leftpower, double rightpower){
        rightFalcon1.set(ControlMode.PercentOutput, rightpower);
        rightFalcon2.set(ControlMode.PercentOutput, rightpower);
        leftFalcon1.set(ControlMode.PercentOutput, leftpower);
        leftFalcon2.set(ControlMode.PercentOutput, leftpower);
    }

    public void stopDrive(){
        rightFalcon1.set(ControlMode.PercentOutput, 0);
        rightFalcon2.set(ControlMode.PercentOutput, 0);
        leftFalcon1.set(ControlMode.PercentOutput, 0);
        leftFalcon2.set(ControlMode.PercentOutput, 0);
    }

    public double getLeftEncoder() {
        return (leftFalcon1.getSelectedSensorPosition() * Constants.DISTANCE_PER_PULSE) / (ShiftGearsSubsystem.getShifterPosition() == ShiftGearsSubsystem.ShiftStatus.HIGH ? Constants.HIGH_GEAR_RATIO : Constants.LOW_GEAR_RATIO);
    }

    public double getRightEncoder() {
        return (rightFalcon1.getSelectedSensorPosition() * Constants.DISTANCE_PER_PULSE) / (ShiftGearsSubsystem.getShifterPosition() == ShiftGearsSubsystem.ShiftStatus.HIGH ? Constants.HIGH_GEAR_RATIO : Constants.LOW_GEAR_RATIO);
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public void resetEncoder() {
        leftFalcon1.setSelectedSensorPosition(0);
        rightFalcon1.setSelectedSensorPosition(0);
    }

    public void driveWithMinPower(double leftPower, double rightPower, double minAbsolutePower) {
        double realLeftPower = (leftPower / Math.abs(leftPower)) * Math.max(Math.abs(leftPower), minAbsolutePower);
        double realRightPower = (rightPower / Math.abs(rightPower)) * Math.max(Math.abs(rightPower), minAbsolutePower);
    }

    public void periodic() {
       System.out.println("Right EnDist: " + getRightEncoder());
       System.out.println("Left EnDist: " + getLeftEncoder());
       System.out.println("Average Distance: " + getAverageEncoder());
       System.out.println("Shift Pos: " + ShiftGearsSubsystem.getShifterPosition());
       System.out.println("");
    }
}

