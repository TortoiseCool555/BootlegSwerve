// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class SwerveCommand extends CommandBase {
  /** Creates a new SwerveCommand. */
  NewSwerveDrivetrain drivetrain;
  XboxController controller;
  double pitchInit = 0;
  double rollInit = 0;
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
    pitchInit = Math.toRadians(drivetrain.getPitch());
    rollInit = Math.toRadians(drivetrain.getRoll());
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

    double pitch = Math.toRadians(drivetrain.getPitch()) - pitchInit;
    double roll = Math.toRadians(drivetrain.getRoll()) - rollInit;
    // pitch = pitch < 0 ? pitch + (2*Math.PI) : pitch;
    // roll = roll < 0 ?  roll + (2 * Math.PI) : roll;
    double vecX = Math.abs(Math.cos(roll) + Math.sin(roll) - 1) < 0.01 ? 0 : Math.cos(roll) + Math.sin(roll) - 1;
    double vecY = Math.abs(Math.cos(pitch) + Math.sin(pitch) - 1) < 0.01 ? 0 : Math.cos(pitch) + Math.sin(pitch) - 1;

    double px = Math.cos(pitch) * Math.sin(roll);
    double py = Math.sin(pitch) * Math.cos(roll);
    double pz = -1 * Math.cos(pitch) * Math.cos(roll);
    double mag1 = Math.hypot(px, py);
    double mag2 = Math.sqrt(Math.pow(px,2) + Math.pow(py,2) + Math.pow(pz,2));
    double angleOffground = Math.abs(Math.toRadians(90) - Math.acos(mag1/mag2)) < 0.005 ? 0 : Math.toRadians(90) - Math.acos(mag1/mag2);
    double angleAround = ExtraMath.atanNew(px, py);
    double kConst = 30.0/45.0;
    double xSpd =Math.abs(Math.cos(angleAround)*(angleOffground * kConst)) < 0.01 ? 0 : ExtraMath.clip(Math.cos(angleAround)*(angleOffground * kConst), 0.25);
    double ySpd =Math.abs(Math.sin(angleAround)*(angleOffground * kConst)) < 0.01 ? 0 : ExtraMath.clip(Math.sin(angleAround)*(angleOffground * kConst), 0.25);
    

    vecY *= 1.5;
    //double vecZ = Math.sin(xRoll) + Math.sin(yRoll);

   // double angle = Math.acos(Math.hypot(vecX, vecY) / Math.sqrt((vecX*vecX) + (vecY*vecY) + (vecZ*vecZ)));

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
    SmartDashboard.putNumber("Roll: ", Math.toDegrees(roll));
    SmartDashboard.putNumber("Pitch", Math.toDegrees(pitch));
    SmartDashboard.putNumber("Raw Roll", drivetrain.getRoll());
    SmartDashboard.putNumber("Raw Pitch", drivetrain.getPitch());
    SmartDashboard.putNumber("Yaw", drivetrain.getYaw());
    SmartDashboard.putNumber("x speed", xSpd);
    SmartDashboard.putNumber("y speed", ySpd);
    SmartDashboard.putNumber("Angle, Off", Math.toDegrees(angleOffground));
    SmartDashboard.putNumber("Angle, Around", Math.toDegrees(angleAround));
    //SmartDashboard.putNumber("VecZ", vecZ);
    //SmartDashboard.putNumber("Overall", Math.toDegrees(angle));
    SmartDashboard.putString("Module Angles", drivetrain.getModuleAngles());
    drivetrain.setChassisSpeeds(0 * Constants.MAX_TRANS_METERS_PER_SEC, 
    0 * Constants.MAX_TRANS_METERS_PER_SEC, 
    0 * Constants.MAX_ANG_RAD_PER_SEC);
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
