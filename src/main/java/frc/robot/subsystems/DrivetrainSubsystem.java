package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private TalonFX rightTalon1;
    private TalonFX rightTalon2;
    private TalonFX leftTalon1;
    private TalonFX leftTalon2;

    public DrivetrainSubsystem() {
        rightTalon1 = new TalonFX(Constants.RIGHT_TALON_ONE);
        rightTalon2 = new TalonFX(Constants.RIGHT_TALON_TWO);
        leftTalon1 = new TalonFX(Constants.LEFT_TALON_ONE);
        leftTalon2 = new TalonFX(Constants.LEFT_TALON_TWO);

        rightTalon1.setNeutralMode(NeutralMode.Brake);
        rightTalon2.setNeutralMode(NeutralMode.Brake);
        leftTalon1.setNeutralMode(NeutralMode.Brake);
        leftTalon2.setNeutralMode(NeutralMode.Brake);

        leftTalon1.setInverted(false);
        rightTalon1.setInverted(true);
        rightTalon2.setInverted(true);
    }

    public void drive(double leftpower, double rightpower){
        rightTalon1.set(ControlMode.PercentOutput, rightpower);
        rightTalon2.set(ControlMode.PercentOutput, rightpower);
        leftTalon1.set(ControlMode.PercentOutput, leftpower);
        leftTalon2.set(ControlMode.PercentOutput, leftpower);
    }

    public void stopDrive(){
        rightTalon1.set(ControlMode.PercentOutput, 0);
        rightTalon2.set(ControlMode.PercentOutput, 0);
        leftTalon1.set(ControlMode.PercentOutput, 0);
        leftTalon2.set(ControlMode.PercentOutput, 0);
    }

    public double getLeftEncoder() {
        return (leftTalon1.getSelectedSensorPosition() * Constants.DISTANCE_PER_PULSE) / (ShiftGearsSubsystem.getShifterPosition() == ShiftGearsSubsystem.ShiftStatus.HIGH ? Constants.HIGH_GEAR_RATIO : Constants.LOW_GEAR_RATIO);
    }

    public double getRightEncoder() {
        return (rightTalon1.getSelectedSensorPosition() * Constants.DISTANCE_PER_PULSE) / (ShiftGearsSubsystem.getShifterPosition() == ShiftGearsSubsystem.ShiftStatus.HIGH ? Constants.HIGH_GEAR_RATIO : Constants.LOW_GEAR_RATIO);
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public void resetEncoder() {
        leftTalon1.setSelectedSensorPosition(0);
        rightTalon1.setSelectedSensorPosition(0);
    }

    public void driveWithMinPower(double leftPower, double rightPower, double minAbsolutePower) {
        double realLeftPower = (leftPower / Math.abs(leftPower)) * Math.max(Math.abs(leftPower), minAbsolutePower);
        double realRightPower = (rightPower / Math.abs(rightPower)) * Math.max(Math.abs(rightPower), minAbsolutePower);
    }
}

