// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.Path;
import frc.robot.Point;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class AprilTagCommand extends CommandBase {
  /** Creates a new AprilTagCommand. */
  NewSwerveDrivetrain drive;
  double xDiff;
  double yDiff;
  double rotDiff;
  double x, y, rot;
  List<Point> points = Arrays.asList(new Point(0, 0, 0), new Point(2, 0, 0), new Point(2.5, 2, 0), new Point(0, 0, 0));
  Path path = new Path(new ArrayList<>(points));
  int segNum = 1;

  public AprilTagCommand(NewSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.initialize();
    segNum = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point robotPoint = new Point(drive.getXPose(), drive.getYPose(), drive.getHeadingPose());
    Point target = path.getNextPoint(robotPoint, segNum, 1);
    Point endPoint = path.getSegment(segNum).get(1);
    // xDiff = 3.5 - drive.getXPose();
    // yDiff = -1.5 - drive.getYPose();
    rotDiff = (0 - drive.getHeadingPose());
    double[] velocities = path.getVelocities(robotPoint, segNum, 1);
    xDiff = velocities[0];
    yDiff = velocities[1];
    // rotDiff = velocities[2];
    x = ExtraMath.clip(xDiff * 1.7, Constants.MAX_TRANS_METERS_PER_SEC);
    y = ExtraMath.clip(yDiff * 1.7, Constants.MAX_TRANS_METERS_PER_SEC);
    rot = ExtraMath.clip(Math.toRadians(rotDiff) * 3.5, Constants.MAX_ANG_RAD_PER_SEC);
    SmartDashboard.putNumber("X Pose", drive.getXPose());
    SmartDashboard.putNumber("Y Pose", drive.getYPose());
    SmartDashboard.putNumber("Z Pose", drive.getHeadingPose());
    SmartDashboard.putNumber("X Input", x);
    SmartDashboard.putNumber("Y Input", y);
    SmartDashboard.putNumber("Rot Input", rot);

    SmartDashboard.putNumber("X Point to Travel", target.getX());
    SmartDashboard.putNumber("Y Point to Travel", target.getY());

    double mag = Math.hypot(robotPoint.getX() - endPoint.getX(), robotPoint.getY() - endPoint.getY());
    if(mag < 0.2 && segNum < points.size() - 1) {
      segNum ++;
    }
    drive.setChassisSpeeds(x, y, rot);
    drive.updateOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
