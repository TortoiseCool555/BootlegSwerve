// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ExtraMath;
import frc.robot.Path;
import frc.robot.Point;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class FollowPath extends CommandBase {
  /** Creates a new FollowPath. */
  NewSwerveDrivetrain drivetrain;
  List<Point> points;
  Path path;
  int segNum = 1;
  double mag = 0;
  double rotErr = 0;
  boolean shouldPID = false;
  Field2d field = new Field2d();
  DecimalFormat formatter = new DecimalFormat("0.00");
  public FollowPath(NewSwerveDrivetrain drivetrain, List<Point> points, boolean shouldPID) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.points = points;
    this.shouldPID = shouldPID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path = new Path(new ArrayList<>(points));
    // drivetrain.setYaw(0);
    // drivetrain.resetOdo();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point robotPoint = new Point(drivetrain.getXPose() - drivetrain.getStartingX(), drivetrain.getYPose() - drivetrain.getStartingY(), Math.toRadians(drivetrain.getHeadingPose()));
    mag = Math.hypot(robotPoint.getX() - points.get(segNum).getX(), robotPoint.getY() - points.get(segNum).getY());
    if(mag < 0.5 && segNum < points.size() - 1) {
      segNum++;
    }

    Point wantedPoint = path.getNextPoint(robotPoint, segNum, 1);
    Point finalPoint = path.getSegment(segNum).get(1);
    double[] veloc = path.getVelocities(robotPoint, segNum, 1, shouldPID);
    double x = veloc[0];
    double y = veloc[1];
    double transVeloc = Math.hypot(x, y);
    if(transVeloc > 1.66) {
      x = x * 3.15 / transVeloc;
      y = y * 3.15 / transVeloc;
    }

    rotErr = Math.toDegrees(ExtraMath.simpleAngleError(robotPoint.getAngleRad(), wantedPoint.getAngleRad()));
    double rot = ExtraMath.clip(veloc[2], 3.4);

    drivetrain.setChassisSpeeds(x,y,rot);

    field.setRobotPose(robotPoint.getX(), robotPoint.getY(), Rotation2d.fromRadians(robotPoint.getAngleRad()));
    SmartDashboard.putString("Robot Point", formatter.format(robotPoint.getX())  + ", " + formatter.format(robotPoint.getY()) + ", " + formatter.format(Math.toDegrees(robotPoint.getAngleRad())));
    SmartDashboard.putNumber("Odom Angle Off", rotErr);
    SmartDashboard.putString("Wanted Point", wantedPoint.getX() + ", " + wantedPoint.getY() + ", " + Math.toDegrees(wantedPoint.getAngleRad()));
    SmartDashboard.putString("Final Point", finalPoint.getX() + ", " + finalPoint.getY() + ", " + Math.toDegrees(finalPoint.getAngleRad()));
    SmartDashboard.putString("Velocities", x + ", " + y + ", " + rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setBrake();
    drivetrain.setChassisSpeeds(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return segNum == points.size() - 1 && mag < 0.8 && Math.abs(rotErr) < 3;
  }
}
