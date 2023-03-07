// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.ArrayList;
import java.util.List;

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
  int segNum = 0;
  double mag = 0;
  public FollowPath(NewSwerveDrivetrain drivetrain, List<Point> points) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.points = points;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.initialize();
    path = new Path(new ArrayList<>(points));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point robotPoint = new Point(drivetrain.getXPose(), drivetrain.getYPose(), drivetrain.getAngle());
    mag = Math.hypot(robotPoint.getX() - points.get(segNum).getX(), robotPoint.getY() - points.get(segNum).getY());
    if(mag < 1 && segNum < points.size() - 1) {
      segNum++;
    }
    
    double[] veloc = path.getVelocities(robotPoint, segNum, 6);
    double x = veloc[0];
    double y = veloc[1];
    double transVeloc = Math.hypot(veloc[0], veloc[1]);
    if(transVeloc > 3.5) {
      x = x * 3.5 / transVeloc;
      y = y * 3.5 / transVeloc;
    }

    double rot = ExtraMath.clip(veloc[2], 9);

    drivetrain.setChassisSpeeds(x,y,rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return segNum < points.size() - 1 && mag < 1;
  }
}
