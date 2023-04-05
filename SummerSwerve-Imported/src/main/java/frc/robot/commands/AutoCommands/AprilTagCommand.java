// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  List<Point> points = Arrays.asList(new Point(0, 2, 0), new Point(0, 4, 0), new Point(5, 4, 0), new Point(5,2,0), new Point(0, 2, 0));
  Path path = new Path(new ArrayList<>(points));
  int segNum = 1;
  final double startingRad = 0.05;
  double currentRad = startingRad;
  final double maxRad = 1;
  Field2d field = new Field2d();

  public AprilTagCommand(NewSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.initialize();
    drive.resetOdo(0, 0);
    segNum = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point robotPoint = new Point(drive.getXPose(), drive.getYPose(), Math.toRadians(drive.getHeadingPose()));
    double mag = Math.hypot(robotPoint.getX() - points.get(segNum).getX(), robotPoint.getY() - points.get(segNum).getY());
    if(mag < 0.5 && segNum < points.size() - 1) {
      segNum++;
      currentRad = startingRad;
    }
    else if(mag < 0.325 && segNum == points.size() - 1){
      cancel();
    }
    currentRad = ExtraMath.clip(currentRad + 1, maxRad);

    Point target = path.getNextPoint(robotPoint, segNum, currentRad);
    double[] velocities = path.getVelocities(robotPoint, segNum, currentRad, true);
    xDiff = velocities[0];
    yDiff = velocities[1];
    rotDiff = velocities[2];
    x = xDiff;
    y = yDiff;
    double overallVeloc = Math.hypot(x, y);
    if(overallVeloc > 3.5) {
      x = x * 3.5 / overallVeloc;
      y = y * 3.5 / overallVeloc;
    }
    rot = ExtraMath.clip(rotDiff, 9);
    
    field.setRobotPose(robotPoint.getX(), robotPoint.getY(), Rotation2d.fromRadians(robotPoint.getAngleRad()));
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("X Pose", drive.getXPose());
    SmartDashboard.putNumber("Y Pose", drive.getYPose());
    SmartDashboard.putNumber("Z Pose", drive.getHeadingPose());
    SmartDashboard.putNumber("X Input", x);
    SmartDashboard.putNumber("Y Input", y);
    SmartDashboard.putNumber("Rot Input", rot);

    SmartDashboard.putNumber("X Point to Travel", target.getX());
    SmartDashboard.putNumber("Y Point to Travel", target.getY());
    SmartDashboard.putNumber("Current Scanning Radius", currentRad);

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
