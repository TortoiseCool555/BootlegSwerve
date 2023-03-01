// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class SwerveCommand extends CommandBase {
  /** Creates a new SwerveCommand. */
  NewSwerveDrivetrain drivetrain;
  XboxController controller;
  public SwerveCommand(XboxController controller, NewSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -controller.getLeftX() * 0.25;
    double y = -controller.getLeftY() * 0.25;
    double rot = -controller.getRightX() * 0.25;

    if(Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(rot) < 0.1) {
      x = 0;
      y = 0;
      rot = 0;
    }


    drivetrain.setChassisSpeeds(y * Constants.MAX_TRANS_METERS_PER_SEC, 
    x * Constants.MAX_TRANS_METERS_PER_SEC, 
    rot * Constants.MAX_ANG_RAD_PER_SEC);

    double xRoll = Math.toRadians(drivetrain.getPitch());
    double yRoll = Math.toRadians(drivetrain.getRoll());
    double vecX = Math.cos((yRoll));
    double vecY = Math.cos((xRoll));
    double vecZ = Math.sin(xRoll) + Math.sin(yRoll);

    double angle = Math.acos(Math.hypot(vecX, vecY) / Math.sqrt((vecX*vecX) + (vecY*vecY) + (vecZ*vecZ)));

    SmartDashboard.putNumber("Yaw Angle", drivetrain.getAngle());
    SmartDashboard.putString("Module Angle Position Values", drivetrain.getModulePositionErrors());
    SmartDashboard.putString("Module Translation Positions", drivetrain.getModuleTranslationPositions());
    SmartDashboard.putString("Module Translation Velocities", drivetrain.getModuleVelocities());
    SmartDashboard.putString("Wanted Translation Velocities", drivetrain.getModuleWantedTranslationVelocity(x,y,rot));
    SmartDashboard.putString("Module Angular Power: ", drivetrain.getModulePositionPowers(x,y,rot));
    SmartDashboard.putString("Module Position Distance", drivetrain.displayModulePositionDist());
    SmartDashboard.putString("Module Position Angle", drivetrain.displayModulePositionAng());
    SmartDashboard.putString("X Stick", Double.toString(x));
    SmartDashboard.putString("Y Stick", Double.toString(y));
    SmartDashboard.putString("Rot Stick", Double.toString(rot));
    SmartDashboard.putString("X", drivetrain.x());
    SmartDashboard.putString("Y", drivetrain.y());
    SmartDashboard.putString("Z", drivetrain.z());
    SmartDashboard.putNumber("Roll: ", drivetrain.getRoll());
    SmartDashboard.putNumber("Pitch", drivetrain.getPitch());
    SmartDashboard.putNumber("VecX", vecX);
    SmartDashboard.putNumber("Overall", Math.toDegrees(angle));

    drivetrain.updateOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
