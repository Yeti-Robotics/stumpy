/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    //Drivetrain constants
    public static final int RIGHT_FALCON_ONE = 14;
    public static final int RIGHT_FALCON_TWO = 7;
    public static final int LEFT_FALCON_ONE = 6;
    public static final int LEFT_FALCON_TWO = 3;

    public static final int LEFT_ENCODER_A = 0;
    public static final int LEFT_ENCODER_B = 1;
    public static final int RIGHT_ENCODER_A = 2;
    public static final int RIGHT_ENCODER_B = 3;

    public static final int DRIVER_STATION_JOY = 0;

    public static final double LOW_GEAR_RATIO = 15.0;
    public static final double HIGH_GEAR_RATIO = 5.13;

    public static final int PULSE_PER_REV = 2048;
    public static final double WHEEL_DIAMETER = 4.0;

    public static final double DISTANCE_PER_PULSE =  (WHEEL_DIAMETER * Math.PI) / PULSE_PER_REV;

    //pathweaver (all avalues are place holders)
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kTrackwidthMeters = 0.69;
    // public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // tunneee
    public static final double kPDriveVel = 8.5;

    //shifter
    public static final int[] SHIFTER_SOLENOID = {0,1};
}