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
    public final static double wallDist = -0.3; // 0.3
    public final static double wallDistCollect = -0.1;
    public final static double wallDistCollectRight = -0.09;
    public final static double rightXErr = -0.4;
    public final static double rightYErr = .4;
    public final static List<Point> DrivePreload = Arrays.asList(new Point(1, 1, 1), new Point(1, 1, 1), new Point(1, 1, 1),
    new Point(1, 1, 1));
    public final static List<Point> CollectExtraFromPreload = Arrays.asList(new Point(1, 1, 1), new Point(1, 1, 1), new Point(1, 1, 1),
    new Point(1, 1, 1));
    public final static List<Point> ScoreExtra = Arrays.asList(new Point(1, 1, 1), new Point(1, 1, 1), new Point(1, 1, 1),
    new Point(1, 1, 1));
    public final static List<Point> SmC = Arrays.asList(new Point(0, 0.0, Math.toRadians(0)), new Point(5.65, 2.0, Math.toRadians(90)));

    public final static List<Point> ScoreToCollectInBetween = Arrays.asList(new Point(0, 0.0, Math.toRadians(180)), new Point(0.5, wallDist, Math.toRadians(0)));
    public final static List<Point> ScoreToCollect = Arrays.asList(new Point(0, wallDist, Math.toRadians(0)), new Point(5.45, wallDistCollectRight, Math.toRadians(0)));
    public final static List<Point> CollectToScoreInBetween = Arrays.asList(new Point(5.38, wallDistCollectRight, Math.toRadians(0)), new Point(3.3, wallDist, Math.toRadians(180)));
    public final static List<Point> CollectToScore = Arrays.asList(new Point(5.38, wallDist, Math.toRadians(180)), new Point(-0.4, wallDist + 0.4, Math.toRadians(180)));
    public final static List<Point> ScoreToCollect2Inbetween = Arrays.asList(new Point(0 + rightXErr, wallDist + rightYErr, Math.toRadians(0)), new Point(5.45 + rightXErr, wallDistCollectRight + rightYErr, Math.toRadians(45)));
    public final static List<Point> ScoreToCollect2 = Arrays.asList(new Point(5.45 + rightXErr, wallDistCollectRight + rightYErr, Math.toRadians(45)), new Point(5.45 + rightXErr, wallDistCollectRight + rightYErr + 1.2, Math.toRadians(45)));
    public final static List<Point> CollectToScore2Inbeween = Arrays.asList(new Point(5.45 + rightXErr, wallDistCollectRight + rightYErr + 1.2, Math.toRadians(45)), new Point(5.45 + rightXErr, wallDistCollectRight + rightYErr, Math.toRadians(60)));
    public final static List<Point> CollectToScore2 = Arrays.asList(new Point(5.45 + rightXErr, wallDistCollectRight + rightYErr, Math.toRadians(60)), new Point(0 + rightXErr, rightYErr, Math.toRadians(180)));

    public final static List<Point> ScoreToCollectInBetweenLeft = Arrays.asList(new Point(0, 0.0, Math.toRadians(180)), new Point(0.5, -wallDist, Math.toRadians(0)));
    public final static List<Point> ScoreToCollectLeft = Arrays.asList(new Point(0, -wallDist, Math.toRadians(0)), new Point(5.45, -wallDistCollect, Math.toRadians(0)));
    public final static List<Point> CollectToScoreInBetweenLeft = Arrays.asList(new Point(5.38, -wallDistCollect, Math.toRadians(0)), new Point(3.3, -wallDist, Math.toRadians(180)));
    public final static List<Point> CollectToScoreLeft = Arrays.asList(new Point(5.38, -wallDist, Math.toRadians(180)), new Point(-0.4, -wallDist - 0.5, Math.toRadians(180)));
    public final static List<Point> ScoreToCollect2InbetweenLeft = Arrays.asList(new Point(0 + rightXErr, -wallDist - rightYErr, Math.toRadians(0)), new Point(5.45 + rightXErr, -wallDistCollect - rightYErr, Math.toRadians(315)));
    public final static List<Point> ScoreToCollect2Left = Arrays.asList(new Point(5.45 + rightXErr, -wallDistCollect - rightYErr, Math.toRadians(315)), new Point(5.45 + rightXErr, -wallDistCollect - rightYErr - 1.2, Math.toRadians(315)));
    public final static List<Point> CollectToScore2InbeweenLeft = Arrays.asList(new Point(5.45 + rightXErr, -wallDistCollect - rightYErr - 1.2, Math.toRadians(330)), new Point(5.45 + rightXErr, -wallDistCollect - rightYErr, Math.toRadians(330)));
    public final static List<Point> CollectToScore2Left = Arrays.asList(new Point(5.45 + rightXErr, -wallDistCollect - rightYErr, Math.toRadians(330)), new Point(-0.4 + rightXErr, -rightYErr, Math.toRadians(180)));
    // 0261

    public final static List<Point> BumpScoreToCollectInBetweenLeft = Arrays.asList(new Point(0, 0.0, Math.toRadians(180)), new Point(5, -wallDist, Math.toRadians(180)));
    // public final static List<Point> BumpScoreToCollectInBetweenLeft2 = Arrays.asList(new Point(0, -wallDist, Math.toRadians(180)), new Point(5, -wallDistCollect, Math.toRadians(0)));
    public final static List<Point> BumpRotate = Arrays.asList(new Point(0, -wallDist, Math.toRadians(0)), new Point(5, -wallDistCollect, Math.toRadians(0)));
    public final static List<Point> BumpScoreToCollectLeft = Arrays.asList(new Point(0, -wallDist, Math.toRadians(0)), new Point(5.45, -wallDistCollect, Math.toRadians(0)));

}