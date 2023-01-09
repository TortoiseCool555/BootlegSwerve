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

    public final static double TRACKWIDTH = 0.5715;
    public final static double WHEELBASE = 0.5842;

    public final static double WHEEL_DIAM = .10033;
    public final static double DRIVING_GEAR_RATIO = (14.0/50.0)*(25.0/19.0)*(15.0/45.0);
    public final static double STEERING_GEAR_RATIO = (15.0/32.0)*(10.0/16.0);
    public final static double ROTATIONS_PER_MINUTE = 6380;
    public final static double MAX_TRANS_METERS_PER_SEC = ROTATIONS_PER_MINUTE / 60 *  WHEEL_DIAM * DRIVING_GEAR_RATIO * Math.PI; 
    public final static double MAX_ANG_RAD_PER_SEC = MAX_TRANS_METERS_PER_SEC / (Math.hypot(TRACKWIDTH / 2, WHEELBASE / 2));

    public final static double TRANS_SCALER = (WHEEL_DIAM * Math.PI) / (TICKS_PER_REVOLUTION * DRIVING_GEAR_RATIO);

    public final static double MAX_PERSONAL_ROT_PER_SEC = 6380 / 60;

    public final static double pRot = 1.0/180.0;
}
