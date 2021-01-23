package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private TalonFX rightFalcon1;
    private TalonFX rightFalcon2;
    private TalonFX leftFalcon1;
    private TalonFX leftFalcon2;
    private ADIS16448_IMU gyro;
    private WPI_TalonFX testFalcon;
    
    private final DifferentialDriveOdometry m_odometry;

    // private final Victor victorTest;
    // private final DifferentialDrive drive;
    public static final DifferentialDrive drive = new DifferentialDrive(new WPI_TalonFX(7), new WPI_TalonFX(7));

    

    public DrivetrainSubsystem() {
        rightFalcon1 = new TalonFX(Constants.RIGHT_FALCON_ONE);
        rightFalcon2 = new TalonFX(Constants.RIGHT_FALCON_TWO);
        leftFalcon1 = new TalonFX(Constants.LEFT_FALCON_ONE);
        leftFalcon2 = new TalonFX(Constants.LEFT_FALCON_TWO);
        // testFalcon = new WPI_TalonFX(7);


        gyro = new ADIS16448_IMU();
        gyro.calibrate();

        rightFalcon1.setNeutralMode(NeutralMode.Brake);
        rightFalcon2.setNeutralMode(NeutralMode.Brake);
        leftFalcon1.setNeutralMode(NeutralMode.Brake);
        leftFalcon2.setNeutralMode(NeutralMode.Brake);

        leftFalcon1.setInverted(false);
        leftFalcon1.setInverted(false);
        rightFalcon1.setInverted(true);
        rightFalcon2.setInverted(true);
        
        // victorTest = new Victor(0);
        // drive = new DifferentialDrive(victorTest,new Victor(1));

        // m_robotDrive = new DifferentialDrive(testFalcon, testFalcon);

        leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
        rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);

        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getAngle(){
        return gyro.getAngle();
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

    //trajectory code gotten from "https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html"
    public void periodic() {
    //    System.out.println("Right EnDist: " + getRightEncoder());
    //    System.out.println("Left EnDist: " + getLeftEncoder());
    //    System.out.println("Average Distance: " + getAverageEncoder());
    //    System.out.println("Shift Pos: " + ShiftGearsSubsystem.getShifterPosition());
    //    System.out.println("");
    //System.out.println("gyro value mayb: " + gyro.getAngle());

        // Update the odometry in the periodic block
        m_odometry.update(gyro.getRotation2d(), leftFalcon1.getSelectedSensorPosition(), rightFalcon1.getSelectedSensorPosition());
    }

     /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        //oroginal method is getRate() from encoder class
        return new DifferentialDriveWheelSpeeds(rightFalcon1.getSelectedSensorVelocity(), leftFalcon1.getSelectedSensorVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoder();
        m_odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        //there is no setvoltage() so i basically translated source code into what can be passed into talonFX
        leftFalcon1.set(ControlMode.PercentOutput, leftVolts/ RobotController.getBatteryVoltage());
        rightFalcon1.set(ControlMode.PercentOutput, -rightVolts/ RobotController.getBatteryVoltage());
        //resets watchdog timer for motor saftey,idk how to fix
        // drive.feed();
    }


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
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }
}

