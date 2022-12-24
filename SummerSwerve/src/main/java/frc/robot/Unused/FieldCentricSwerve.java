// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Unused;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ExtraMath;

public class FieldCentricSwerve extends CommandBase {
  /** Creates a new FieldCentricSwerve. */
  SwerveDrivetrain swerve;
  XboxController controller;
  public FieldCentricSwerve(SwerveDrivetrain swerve, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX = ExtraMath.clip(ExtraMath.exponential(controller.getLeftX(), .11, 0), .1); // .25 max .46a .05 min x 
    // double magnitude = Math.sqrt(Math.pow(controller.getLeftX(), 2) + Math.pow(controller.getLeftY(), 2));
    // double angle = ExtraMath.atanNew(controller.getLeftY() , controller.getLeftX());
    // double turn = controller.getRightX();

    // swerve.setRotMotors(leftX);
    SmartDashboard.putNumber("Module 1", swerve.getModule1Deg());
    SmartDashboard.putNumber("Module 2", swerve.getModule2Deg());
    SmartDashboard.putNumber("Module 3", swerve.getModule3Deg());
    SmartDashboard.putNumber("Module 4", swerve.getModule4Deg());

    // swerve.setMotors(magnitude, angle, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
