// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.commands.SwerveCommand;

public class NewSwerveDrivetrain extends SubsystemBase {
  /** Creates a new NewSwerveDrivetrain. */
  XboxController controller;

  NewSwerveModule lfModule = new NewSwerveModule(12, 11, 10);
  NewSwerveModule lbModule = new NewSwerveModule(3, 2, 1);
  NewSwerveModule rfModule = new NewSwerveModule(9, 8, 7);
  NewSwerveModule rbModule = new NewSwerveModule(6, 5, 4);

  private Pigeon2 gyro = new Pigeon2(13);

  boolean stalled = false;

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(Constants.TRACKWIDTH/2,Constants.WHEELBASE/2), new Translation2d(Constants.TRACKWIDTH/2,-Constants.WHEELBASE/2), 
  new Translation2d(-Constants.TRACKWIDTH/2,Constants.WHEELBASE/2), new Translation2d(-Constants.TRACKWIDTH/2,-Constants.WHEELBASE/2));

  public NewSwerveDrivetrain(XboxController controller) {
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new SwerveCommand(controller, this));
  }

  public void init() {
    lfModule.initialize();
    lbModule.initialize();
    rfModule.initialize();
    rbModule.initialize();
  }

  public void setChassisSpeeds(double velocityX, double velocityY, double angularVelocity) {
    double angle = getAngle();

    ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, Rotation2d.fromDegrees(angle));

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeed);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_TRANS_PER_SEC);

    if(velocityX == 0 && velocityY == 0 && angularVelocity == 0) {
      stalled = true;
    } else {
      stalled = false;
    }
    lfModule.set(moduleStates[0], angle, stalled);
    rfModule.set(moduleStates[1], angle, stalled);
    lbModule.set(moduleStates[2], angle, stalled);
    rbModule.set(moduleStates[3], angle, stalled);
  }

  public double getAngle() {
    return ExtraMath.mod(gyro.getYaw(), 360);
  }

  // Value Prints

  public void ZeroGyro() {
    gyro.setYaw(0);
  }

  public String getModulePositionErrors() {
    return lfModule.getPositionError() + " " + lbModule.getPositionError() + " " + rfModule.getPositionError() + " " + rbModule.getPositionError();
  }
  
  public String getModulePositionPowers(double velocityX, double velocityY, double angularVelocity) {
    double angle = getAngle();
    ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, Rotation2d.fromDegrees(angle));

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_TRANS_PER_SEC);
    return lfModule.getAnglePowerExperimental(moduleStates[0]) + " " + lbModule.getAnglePowerExperimental(moduleStates[2]) + " " + rfModule.getAnglePowerExperimental(moduleStates[1]) + " " + rbModule.getAnglePowerExperimental(moduleStates[3]);
  }

  // Other
  public String getModuleVelocities() {
    return lfModule.getVelocity() + " " + lbModule.getVelocity() + " " + rfModule.getVelocity() + " " + rbModule.getVelocity();
  }

  public String getModuleTranslationPositions() {
    return lfModule.getTransPosition() + " " + lbModule.getTransPosition() + " " + rfModule.getTransPosition() + " " + rbModule.getTransPosition();
  }
}
