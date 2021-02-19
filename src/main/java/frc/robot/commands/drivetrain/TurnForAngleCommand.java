package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class TurnForAngleCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double leftPower;
    private double rightPower;
    private double gyroGoal;

    public TurnForAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double gyroGoal, double leftPower, double rightPower) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.gyroGoal = gyroGoal;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.resetGyro();
    }

    @Override
    public void execute() {
        if(gyroGoal < 0){
            drivetrainSubsystem.drive(-leftPower, rightPower);
        } else {
            drivetrainSubsystem.drive(leftPower, -rightPower);
        }
    }

    @Override
    public boolean isFinished() {
        return (gyroGoal >= this.drivetrainSubsystem.getAngle()-1 && gyroGoal <= this.drivetrainSubsystem.getAngle()+1);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopDrive();
    }
}
