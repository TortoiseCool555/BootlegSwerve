// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

/** Add your docs here. */
public class Paths {
    // Sn = Score at n, can be St, Sm, Sl, top, mid, low
    // C = charge station
    // Gn = Grab at n, can be Gt, Gm, Gl, top, mid, low
    public final static List<Point> DrivePreload = Arrays.asList(new Point(1, 1, 1), new Point(1, 1, 1), new Point(1, 1, 1),
    new Point(1, 1, 1));
    public final static List<Point> CollectExtraFromPreload = Arrays.asList(new Point(1, 1, 1), new Point(1, 1, 1), new Point(1, 1, 1),
    new Point(1, 1, 1));
    public final static List<Point> ScoreExtra = Arrays.asList(new Point(1, 1, 1), new Point(1, 1, 1), new Point(1, 1, 1),
    new Point(1, 1, 1));
    public final static List<Point> SmC = Arrays.asList(new Point(0, 0.0, Math.toRadians(180)), new Point(5.65, 0.01, Math.toRadians(180)));
}