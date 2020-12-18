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
    public static final int RIGHT_TALON_ONE = 0;
    public static final int RIGHT_TALON_TWO = 1;
    public static final int LEFT_TALON_ONE = 2;
    public static final int LEFT_TALON_TWO = 3;

    public static final int LEFT_ENCODER_A = 0;
    public static final int LEFT_ENCODER_B = 1;
    public static final int RIGHT_ENCODER_A = 2;
    public static final int RIGHT_ENCODER_B = 3;

    public static final int DISTANCE_PER_PULSE = 2048;

    public static final int DRIVER_STATION_JOY = 0;

    public static final double LOW_GEAR_RATIO = 12/60;
    public static final double HIGH_GEAR_RATIO = 30/44;

    //shifter
    public static final int[] SHIFTER_SOLENOID = {0,1};
}