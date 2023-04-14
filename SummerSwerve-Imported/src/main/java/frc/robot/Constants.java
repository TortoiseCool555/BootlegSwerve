// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static double ticksInRotation = 3;
    public final static double translationMin = 0.1;
    public final static double rotationalPVal = 0.01;

    public final static double TICKS_PER_REVOLUTION = 2048; //4096

    public final static double through_bore_TPR = 8192.0;
    public final static double TRACKWIDTH = 0.535;
    public final static double WHEELBASE = 0.59;

    public final static double WHEEL_DIAM = .10033;
    public final static double WHEEL_CIRCUM = WHEEL_DIAM * Math.PI;
    public final static double DRIVING_GEAR_RATIO = 1.0/6.75;
    public final static double STEERING_GEAR_RATIO = 1.0/12.8;
    public final static double ROTATIONS_PER_MINUTE = 6380;
    public final static double MAX_TRANS_METERS_PER_SEC = ROTATIONS_PER_MINUTE / 60 *  WHEEL_DIAM * DRIVING_GEAR_RATIO * Math.PI; 
    public final static double MAX_ANG_RAD_PER_SEC = MAX_TRANS_METERS_PER_SEC / (Math.hypot(TRACKWIDTH / 2, WHEELBASE / 2));

    public final static double TRANS_SCALER = (WHEEL_DIAM * Math.PI) / (TICKS_PER_REVOLUTION * DRIVING_GEAR_RATIO);

    public final static double MAX_PERSONAL_ROT_PER_SEC = 6380 / 60;

    public final static double pRot = 0.005;
    public static final String grabMode = null;

    public static boolean scoringMode = false;
    public static double elevatorHeight = 0;

    public static final double EL_HEIGHT_OFF_GROUND = .30;
    public static final double ELEVATOR_TO_METERS = .78 / 9500; // Inches over ticks
    public static final double EXTENSION_TO_METERS = .52 / 17.2; // Inches over ticks
    public static final double DIST_FROM_SCORE = 0.361;
    public static final double HEIGHT_FOR_SCORE = 0.597;
    public static final double HEIGHT_FOR_SCORE2 = 0.9017;
    public static final double DIST_FOR_CUBE = 0.44;
    public static final double ARM_LENGTH = .5;
}
