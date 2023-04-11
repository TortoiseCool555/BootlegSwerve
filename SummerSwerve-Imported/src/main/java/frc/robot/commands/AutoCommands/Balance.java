// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class Balance extends CommandBase {
  /** Creates a new Balance. */
  NewSwerveDrivetrain drivetrain;
  double angleOffground;
  public Balance(NewSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.ZeroGyro();
    double pitch = Math.toRadians(drivetrain.getPitch());
    double roll = Math.toRadians(drivetrain.getRoll());
    double px = Math.cos(pitch) * Math.sin(roll);
    double py = Math.sin(pitch) * Math.cos(roll);
    double pz = -1 * Math.cos(pitch) * Math.cos(roll);
    double mag1 = Math.hypot(px, py);
    double mag2 = Math.sqrt(Math.pow(px,2) + Math.pow(py,2) + Math.pow(pz,2));
    angleOffground = Math.abs(Math.toRadians(90) - Math.acos(mag1/mag2)) < 0.005 ? 0 : Math.toRadians(90) - Math.acos(mag1/mag2);
    double angleAround = ExtraMath.atanNew(px, py);
    double kConst = 1.7; //2.15
    double xSpd = Math.abs(Math.cos(angleAround)*(angleOffground * kConst)) < 0.01 ? 0 : ExtraMath.clip(Math.cos(angleAround)*(angleOffground * kConst), 0.5);
    double ySpd = Math.abs(Math.sin(angleAround)*(angleOffground * kConst)) < 0.01 ? 0 : ExtraMath.clip(Math.sin(angleAround)*(angleOffground * kConst), 0.5);
    drivetrain.setChassisSpeeds(xSpd, ySpd, 0);

    SmartDashboard.putNumber("x speed", xSpd * Constants.MAX_TRANS_METERS_PER_SEC);
    SmartDashboard.putNumber("y speed", ySpd * Constants.MAX_TRANS_METERS_PER_SEC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(0, 0, 0.01);
    drivetrain.setChassisSpeeds(0, 0, 0.0);
    drivetrain.setYaw(180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleOffground) < Math.toRadians(10);
  }
}
