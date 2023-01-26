// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class AprilTagCommand extends CommandBase {
  /** Creates a new AprilTagCommand. */
  NewSwerveDrivetrain drive;
  SwerveControllerCommand test;
  double xDiff;
  double yDiff;
  double rotDiff;
  double x, y, rot;
  
  public AprilTagCommand(NewSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.initialize();
    State fState = new State(2, 2, 2, new Pose2d(1.5, -1.5, Rotation2d.fromDegrees(0)), 1);
    State sState = new State(2, 2, 2, new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 3);
    State tState = new State(2, 2, 2, new Pose2d(0, -2, Rotation2d.fromDegrees(90)), 1);
    List<State> states = Arrays.asList(fState, sState,tState);
    Trajectory traj = new Trajectory(states);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xDiff = 3.5 - drive.getXPose();
    yDiff = -1.5 - drive.getYPose();
    rotDiff = (90 - drive.getHeadingPose());
    x = ExtraMath.clip(xDiff * 1.55, Constants.MAX_TRANS_METERS_PER_SEC);
    y = ExtraMath.clip(yDiff * 1.55, Constants.MAX_TRANS_METERS_PER_SEC);
    rot = ExtraMath.clip(Math.toRadians(rotDiff) * 3.5, Constants.MAX_ANG_RAD_PER_SEC);
    SmartDashboard.putNumber("X Pose", drive.getXPose());
    SmartDashboard.putNumber("Y Pose", drive.getYPose());
    SmartDashboard.putNumber("Z Pose", drive.getHeadingPose());
    SmartDashboard.putNumber("X Input", x);
    SmartDashboard.putNumber("Y Input", y);
    SmartDashboard.putNumber("Rot Input", rot);

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
