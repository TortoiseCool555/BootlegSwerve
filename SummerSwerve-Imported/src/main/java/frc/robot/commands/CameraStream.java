// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;

public class CameraStream extends CommandBase {
  /** Creates a new CameraStream. */
  CameraSubsystem cam;
  public CameraStream(CameraSubsystem cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cam);
    this.cam = cam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cam.setDaemon();
    cam.startVision();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cam.stopVision();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
