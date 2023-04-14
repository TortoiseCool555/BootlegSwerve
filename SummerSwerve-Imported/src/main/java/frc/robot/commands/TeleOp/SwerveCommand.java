// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOp;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class SwerveCommand extends CommandBase {
  /** Creates a new SwerveCommand. */
  NewSwerveDrivetrain drivetrain;
  XboxController controller;
  double pitchInit = 0;
  double rollInit = 0;
  double prevX = 0;
  double prevY = 0;
  boolean disablePidgeon = false;
  double startX = 1;
  double startY = 1;
  double startAngle = 1;
  public SwerveCommand(XboxController controller, NewSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivetrain.setChassisSpeeds(0, 0, 0);
    drivetrain.initialize();
    // drivetrain.setChassisSpeeds(0, 0, 0);
    // drivetrain.setYaw(0);
    // drivetrain.getYaw();
    // drivetrain.resetOdo(0, 0);
    // drivetrain.updateOdometry();
    // drivetrain.setChassisSpeeds(0.01, 0, 0);
    // drivetrain.updateOdometry();
    startX = drivetrain.getXPose();
    startY = drivetrain.getYPose();
    // startX = drivetrain.getXPose();
    // startY = drivetrain.getYPose();
    // startX = drivetrain.getXPose();
    // startY = drivetrain.getYPose();
    // startX = drivetrain.getXPose();
    // startY = drivetrain.getYPose();
    // startX = drivetrain.getXPose();
    // startY = drivetrain.getYPose();
    // startAngle = drivetrain.getYaw();
    // drivetrain.resetOdo(0, 0);
    // pitchInit = Math.toRadians(drivetrain.getPitch());
    // rollInit = Math.toRadians(drivetrain.getRoll());
    pitchInit = 0;
    rollInit = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double scale = 1.0; //0.37
    double rotScale = 0.75;
    double speedToCoast = 3.5;
    if(controller.getRightTriggerAxis() > 0.1) {
      scale = 0.13;
      rotScale = 0.1;
    }
    if(Math.abs(Constants.elevatorHeight) > 4000) {
      scale = 0.13;
      rotScale = 0.1;
      speedToCoast = 1;
    }
    double x = controller.getLeftX() * scale;
    double y = -controller.getLeftY() * scale;
    double rot = -controller.getRightX() * rotScale;

    if(controller.getYButtonPressed()) {
      disablePidgeon = true;
    } else if(controller.getAButtonPressed()) {
      disablePidgeon = false;
    }

    if(disablePidgeon) {

    } else {

    }

    if(Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(rot) < 0.1) {
      // x = prevX / 4;
      // y = prevY / 4;
      x = 0;
      y = 0;
      rot = 0;
    }

    if(Math.abs(drivetrain.driveSpeed()) < speedToCoast && (x == 0 && y == 0 && rot == 0)){
      drivetrain.setBrake();
    }
    else{
      drivetrain.setCoast();
    }

    if(controller.getLeftBumper()) {
      Constants.scoringMode = false;
    } else if(controller.getRightBumper()) {
      Constants.scoringMode = true;
    }

    // double pitch = Math.toRadians(drivetrain.getPitch()) - pitchInit;
    // double roll = Math.toRadians(drivetrain.getRoll()) - rollInit;
    // pitch = pitch < 0 ? pitch + (2*Math.PI) : pitch;
    // roll = roll < 0 ?  roll + (2 * Math.PI) : roll;
    // double px = Math.cos(pitch) * Math.sin(roll);
    // double py = Math.sin(pitch) * Math.cos(roll);
    // double pz = -1 * Math.cos(pitch) * Math.cos(roll);
    // double mag1 = Math.hypot(px, py);
    // double mag2 = Math.sqrt(Math.pow(px,2) + Math.pow(py,2) + Math.pow(pz,2));
    // double angleOffground = Math.abs(Math.toRadians(90) - Math.acos(mag1/mag2)) < 0.005 ? 0 : Math.toRadians(90) - Math.acos(mag1/mag2);
    // double angleAround = ExtraMath.atanNew(px, py);
    // double kConst = 2;
    // double xSpd =Math.abs(Math.cos(angleAround)*(angleOffground * kConst)) < 0.01 ? 0 : ExtraMath.clip(Math.cos(angleAround)*(angleOffground * kConst), 0.25);
    // double ySpd =Math.abs(Math.sin(angleAround)*(angleOffground * kConst)) < 0.01 ? 0 : ExtraMath.clip(Math.sin(angleAround)*(angleOffground * kConst), 0.25);


    SmartDashboard.putBoolean("Mode", Constants.scoringMode);
    // SmartDashboard.putNumber("Yaw Angle", drivetrain.getAngle());
    // SmartDashboard.putString("Module Angle Position Values", drivetrain.getModulePositionErrors());
    // SmartDashboard.putString("Module Translation Positions", drivetrain.getModuleTranslationPositions());
    // SmartDashboard.putString("Module Translation Velocities", drivetrain.getModuleVelocities());
    // SmartDashboard.putString("Wanted Translation Velocities", drivetrain.getModuleWantedTranslationVelocity(x,y,rot));
    // SmartDashboard.putString("Module Angular Power: ", drivetrain.getModulePositionPowers(x,y,rot));
    // SmartDashboard.putString("Module Position Distance", drivetrain.displayModulePositionDist());
    // SmartDashboard.putString("Module Position Angle", drivetrain.displayModulePositionAng());
    // SmartDashboard.putString("X Stick", Double.toString(x));
    // SmartDashboard.putString("Y Stick", Double.toString(y));
    // SmartDashboard.putString("Rot Stick", Double.toString(rot));
    SmartDashboard.putNumber("X", drivetrain.getXPose());
    SmartDashboard.putNumber("Y", drivetrain.getYPose());
    SmartDashboard.putNumber("xOff", startX);
    SmartDashboard.putNumber("yOff", startAngle);
    SmartDashboard.putNumber("RawX", drivetrain.getXPose());
    SmartDashboard.putNumber("RawY", drivetrain.getYPose());
    SmartDashboard.putString("Z", drivetrain.z());
    // SmartDashboard.putNumber("Roll: ", Math.toDegrees(roll));
    // SmartDashboard.putNumber("Pitch", Math.toDegrees(pitch));
    SmartDashboard.putNumber("Yaw", drivetrain.getYaw());
    SmartDashboard.putNumber("Raw Roll", drivetrain.getRoll());
    SmartDashboard.putNumber("Raw Pitch", drivetrain.getPitch());
    SmartDashboard.putString("Wheel velocities", drivetrain.getModuleVelocities());
    SmartDashboard.putNumber("Scale value", scale);
    SmartDashboard.putNumber("Speed to Coast", speedToCoast);
    // SmartDashboard.putNumber("x speed", xSpd);
    // SmartDashboard.putNumber("y speed", ySpd);
    // SmartDashboard.putNumber("Angle, Off", Math.toDegrees(angleOffground));
    // SmartDashboard.putNumber("Angle, Around", Math.toDegrees(angleAround));
    // //SmartDashboard.putNumber("Overall", Math.toDegrees(angle));
    // SmartDashboard.putString("Module Angles", drivetrain.getModuleAngles());
    // drivetrain.setChassisSpeeds(0 * Constants.MAX_TRANS_METERS_PER_SEC, 
    // 0 * Constants.MAX_TRANS_METERS_PER_SEC, 
    // 0 * Constants.MAX_ANG_RAD_PER_SEC);
    drivetrain.setChassisSpeeds(y * Constants.MAX_TRANS_METERS_PER_SEC, 
    -x * Constants.MAX_TRANS_METERS_PER_SEC, 
    rot * Constants.MAX_ANG_RAD_PER_SEC);
    prevX = x;
    prevY = y;
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
