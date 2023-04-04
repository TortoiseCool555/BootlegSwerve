// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class InitDrivetrain extends CommandBase {
  /** Creates a new InitDrivetrain. */
  NewSwerveDrivetrain drivetrain;
  double x;
  double y;
  double angleRad;
  public InitDrivetrain(NewSwerveDrivetrain drivetrain, double x, double y, double angleRad) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.x = x;
    this.y = y;
    this.angleRad = angleRad;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.initialize();
    drivetrain.setX(x, y, angleRad);
    drivetrain.setYaw(Math.toDegrees(angleRad));
    // drivetrain.resetOdo();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
