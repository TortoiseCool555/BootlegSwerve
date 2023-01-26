// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryTest extends InstantCommand {
  NewSwerveDrivetrain drive;
  SwerveControllerCommand follower;
  public TrajectoryTest(NewSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveControllerCommand test;
    State fState = new State(2, 2, 2, new Pose2d(1.5, -1.5, Rotation2d.fromDegrees(0)), 1);
    State sState = new State(2, 2, 2, new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 3);
    State tState = new State(2, 2, 2, new Pose2d(0, -2, Rotation2d.fromDegrees(90)), 1);
    List<State> states = Arrays.asList(fState, sState,tState);
    Trajectory traj = new Trajectory(states);
    drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }
}
