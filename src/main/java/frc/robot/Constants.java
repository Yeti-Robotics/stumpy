/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    //Drivetrain constants
    public static final int RIGHT_FALCON_ONE = 3;
    public static final int RIGHT_FALCON_TWO = 8;
    
    public static final int RIGHT_FALCON_THREE = 6;
    public static final int LEFT_FALCON_ONE = 13;
    public static final int LEFT_FALCON_TWO = 1;
    public static final int LEFT_FALCON_THREE = 7;

    public static final int LEFT_ENCODER_A = 0;
    public static final int LEFT_ENCODER_B = 1;
    public static final int RIGHT_ENCODER_A = 2;
    public static final int RIGHT_ENCODER_B = 3;

    public static final int GYRO_TALON = 8;

    public static final int DRIVER_STATION_JOY = 0;

    public static final double LOW_GEAR_RATIO = 15.0;
    public static final double HIGH_GEAR_RATIO = 5.13;

    public static final int PULSE_PER_REV = 4096;
    public static final double WHEEL_DIAMETER = 4.0;

    public static final double DISTANCE_PER_PULSE =  (WHEEL_DIAMETER * Math.PI) / PULSE_PER_REV;

    //limelight PID 
    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    //trajectory tutorial values
    public static final double ksVolts = 0.717;
    public static final double kvVoltSecondsPerMeter = 3.43;
    public static final double kaVoltSecondsSquaredPerMeter = 0.27;

    public static final double kTrackwidthMeters = 2.2677916208956517;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kPDriveVel = 1.88;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;



    //shifter
    public static final int[] SHIFTER_SOLENOID = {0,1};

    //shooter motor constants
    public static final int RIGHT_SHOOTER_MOTOR = 1;
    public static final int LEFT_SHOOTER_MOTOR = 0;

    //distance calc constants
    public static final double KNOWN_DISTANCE = 161.3; //inches
    public static final int PIXEL_WIDTH_KNOWN = 65; //pixels
    public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; //inches
    public static final double FOCAL_LENGTH = ( KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;

    //shooter constants
    public static final int NECK_ROLLER_VICTOR = 99;
    public static final int HOPPER_ROLLER_VICTOR = 98;
    public static final int INTAKE_ROLLER_TALON = 97;
    public static final int[] INTAKE_PISTONS_SOLENOID = {2,3};

    public static final double NECK_IN_SPEED = .75;
    public static final double NECK_OUT_SPEED = -.75;
    public static final double HOPPER_IN_SPEED = 0.5;
    public static final double HOPPER_OUT_SPEED = -0.5;
    public static final double INTAKE_IN_SPEED = 0.5;
    public static final double INTAKE_OUT_SPEED = -0.5;
}