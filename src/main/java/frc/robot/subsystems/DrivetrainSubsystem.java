package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    
    private WPI_TalonSRX rightTalon1;
    private WPI_TalonSRX rightTalon2;
    private WPI_VictorSPX rightVictor3;
    private WPI_TalonSRX leftTalon1;
    private WPI_VictorSPX leftVictor2;
    private WPI_VictorSPX leftVictor3;
    // private WPI_TalonFX testFalcon;

    
    
     private final DifferentialDriveOdometry m_odometry;
    private final WPI_TalonSRX gyroTalon;
    private final PigeonIMU gyro;

    // private final Victor victorTest;
    // private final DifferentialDrive drive;
    public final DifferentialDrive drive;

    private Pose2d pose;
    
    private DriveMode driveMode;

    public enum DriveMode {
        TANK, CHEEZY, ARCADE;
    }

    public DrivetrainSubsystem() {
        rightTalon1 = new WPI_TalonSRX(Constants.RIGHT_FALCON_ONE);
        rightTalon2 = new WPI_TalonSRX(Constants.RIGHT_FALCON_TWO);
        rightVictor3 = new WPI_VictorSPX(Constants.RIGHT_FALCON_THREE);
        leftTalon1 = new WPI_TalonSRX(Constants.LEFT_FALCON_ONE);
        leftVictor2 = new WPI_VictorSPX(Constants.LEFT_FALCON_TWO);
        leftVictor3 = new WPI_VictorSPX(Constants.LEFT_FALCON_THREE);
        
        rightTalon1.setInverted(true);
        rightTalon2.setInverted(true);
        rightVictor3.setInverted(true);

        rightTalon2.follow(rightTalon1);
        rightVictor3.follow(rightTalon1);
        leftVictor2.follow(leftTalon1);
        leftVictor3.follow(leftTalon1);
        
        // drive.setSafetyEnabled(false);

        gyroTalon = new WPI_TalonSRX(Constants.GYRO_TALON);
        gyro = new PigeonIMU(gyroTalon);
        // gyro.calibrate();
        // gyro.reset();
        resetGyro();
           

        
        
        // victorTest = new Victor(0);
        // drive = new DifferentialDrive(victorTest,new Victor(1));

        // m_robotDrive = new DifferentialDrive(testFalcon, testFalcon);

        leftTalon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
        rightTalon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
        resetEncoder();

        leftTalon1.setNeutralMode(NeutralMode.Brake);
        rightTalon1.setNeutralMode(NeutralMode.Brake);
        leftVictor2.setNeutralMode(NeutralMode.Brake);
        rightTalon2.setNeutralMode(NeutralMode.Brake);
        leftVictor3.setNeutralMode(NeutralMode.Brake);
        rightVictor3.setNeutralMode(NeutralMode.Brake);

         m_odometry = new DifferentialDriveOdometry(getHeading());

         this.driveMode = DriveMode.TANK;
        

        
        drive = new DifferentialDrive(leftTalon1, rightTalon1);

    }

    public void resetGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getHeading() {
        double ypr[] = {0,0,0};
        gyro.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0d));
    }

    public double getAngle(){
        double [] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public DriveMode getDriveMode() {
        return this.driveMode;
    }

    // public void testPathWeaver(){
    //     String trajectoryJSON = "paths/Test.wpilib.json";
    //     Trajectory trajectory = new Trajectory(null);
    //     try {
    //          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //         System.out.println("inside try");
    //     } catch (IOException ex) {
    //         DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    //         System.out.println("inside catch");
    //     }
    // }

    public void drive(double leftPower, double rightPower){
        rightTalon1.set(ControlMode.PercentOutput, rightPower);
        rightTalon2.set(ControlMode.PercentOutput, rightPower);
        rightVictor3.set(ControlMode.PercentOutput, rightPower);
        leftTalon1.set(ControlMode.PercentOutput, leftPower);
        leftVictor2.set(ControlMode.PercentOutput, leftPower);
        leftVictor3.set(ControlMode.PercentOutput, leftPower);
    }

    public void stopDrive(){
        rightTalon1.set(ControlMode.PercentOutput, 0);
        leftTalon1.set(ControlMode.PercentOutput, 0);
    }

    public void tankDrive(double leftPower, double rightPower) {
        drive.tankDrive(leftPower, rightPower);
    }

    public void cheezyDrive(double straight, double turn) {
        drive.curvatureDrive(straight, -turn, false);
      }

    public double getLeftEncoder() {
        return (leftTalon1.getSelectedSensorPosition() * Constants.DISTANCE_PER_PULSE) / (ShiftGearsSubsystem.getShifterPosition() == ShiftGearsSubsystem.ShiftStatus.HIGH ? Constants.HIGH_GEAR_RATIO : Constants.LOW_GEAR_RATIO);
    }


    public double getRightEncoder() {
        return (rightTalon1.getSelectedSensorPosition() * Constants.DISTANCE_PER_PULSE) / (ShiftGearsSubsystem.getShifterPosition() == ShiftGearsSubsystem.ShiftStatus.HIGH ? Constants.HIGH_GEAR_RATIO : Constants.LOW_GEAR_RATIO);
    }


    public double getAverageEncoder() {
        System.out.println(getLeftEncoder());
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

    //trajectory code gotten from "https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html"
    public void periodic() {
    //    System.out.print("Right EnDist: " + getRightEncoder());
    //    System.out.println(", Left EnDist: " + getLeftEncoder());
    //    System.out.println("Average Distance: " + getAverageEncoder());
    //    System.out.println("Shift Pos: " + ShiftGearsSubsystem.getShifterPosition());
    //    System.out.println("");
        System.out.println("gyro: " + getAngle());
        drive.feed();
        // Update the odometry in the periodic block
         pose = m_odometry.update(getHeading(), leftTalon1.getSelectedSensorPosition(), rightTalon1.getSelectedSensorPosition());
    }

     /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
     public Pose2d getPose() {
//          return m_odometry.getPoseMeters();
         return pose;
     }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // System.out.println("getting da wheel speeds!");
        //original method is getRate() from encoder class
        return new DifferentialDriveWheelSpeeds(rightTalon1.getSelectedSensorVelocity(), leftTalon1.getSelectedSensorVelocity());
    }

    public void resetOdometry() {
        resetEncoder();
         m_odometry.resetPosition(new Pose2d(), getHeading());
    }

    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    // public void tankDriveVolts(double leftVolts, double rightVolts) {
    //     // System.out.println("tank volt drives getting something");
    //     leftSide.setVoltage(leftVolts);
    //     rightSide.setVoltage(rightVolts);
    //     drive.feed();
    // }


     /**
    * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
    *
    * @param maxOutput the maximum output to which the drive will be constrained
    */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        // gyro.reset();
    }



    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        // return -gyro.getRate();
        return 2.0;
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
      }
}

