// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.commands.SwerveCommand;
public class NewSwerveDrivetrain extends SubsystemBase {
  /** Creates a new NewSwerveDrivetrain. */
  XboxController controller;

  NewSwerveModule lfModule = new NewSwerveModule(6, 5, 4, 0);
  NewSwerveModule lbModule = new NewSwerveModule(9, 8, 7,0);
  NewSwerveModule rfModule = new NewSwerveModule(3, 2, 1,0);
  NewSwerveModule rbModule = new NewSwerveModule(12, 11,10, 0);

  private Pigeon2 gyro = new Pigeon2(13, "CANivoreA");

  boolean stalled = false;
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(Constants.TRACKWIDTH/2,Constants.WHEELBASE/2), new Translation2d(Constants.TRACKWIDTH/2,-Constants.WHEELBASE/2), 
  new Translation2d(-Constants.TRACKWIDTH/2,Constants.WHEELBASE/2), new Translation2d(-Constants.TRACKWIDTH/2,-Constants.WHEELBASE/2));
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getAngle()), new SwerveModulePosition[]{lfModule.getModulePosition(),rfModule.getModulePosition(),lbModule.getModulePosition(),rbModule.getModulePosition()});
  ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(gyro.getYaw()));
  SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
  public NewSwerveDrivetrain(XboxController controller) {
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new SwerveCommand(controller, this));
  }

  public void initialize() {
    gyro.setYaw(0);
    lfModule.initialize();
    lbModule.initialize();
    rfModule.initialize();
    rbModule.initialize();
  }

  /**
   * 
   * @param velocityX - X velocity in m/s
   * @param velocityY - Y velocity in m/s
   * @param angularVelocity - Angular velocity in rad/s
   */
  public void setChassisSpeeds(double velocityX, double velocityY, double angularVelocity) {
    double angle = getAngle();

    ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, Rotation2d.fromDegrees(angle));

    moduleStates = kinematics.toSwerveModuleStates(chassisSpeed);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_TRANS_METERS_PER_SEC);

    if(velocityX == 0 && velocityY == 0 && angularVelocity == 0) {
      stalled = true;
    } else {
      stalled = false;
    }
    lfModule.set(moduleStates[0], stalled);
    rfModule.set(moduleStates[1], stalled);
    lbModule.set(moduleStates[2], stalled);
    rbModule.set(moduleStates[3], stalled);
  }
  public void setStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_TRANS_METERS_PER_SEC);
    lfModule.set(desiredStates[0], false);
    rfModule.set(desiredStates[1], false);
    lbModule.set(desiredStates[2], false);
    rbModule.set(desiredStates[3], false);

  }

  /**
   * Sets gyroscope to 0 degrees
   */
  public void ZeroGyro() {
    gyro.setYaw(0);
  }

  public double getAngle() {
    return ExtraMath.mod(gyro.getYaw(), 360);
  }

  // Value Prints
  public String getModulePositionErrors() {
    return lfModule.getPositionError() + " " + lbModule.getPositionError() + " " + rfModule.getPositionError() + " " + rbModule.getPositionError();
  }
  
  public String getModulePositionPowers(double velocityX, double velocityY, double angularVelocity) {
    double angle = getAngle();
    ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, Rotation2d.fromDegrees(angle));

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_TRANS_METERS_PER_SEC);
    return lfModule.getAnglePowerExperimental(moduleStates[0]) + " " + lbModule.getAnglePowerExperimental(moduleStates[2]) + " " + rfModule.getAnglePowerExperimental(moduleStates[1]) + " " + rbModule.getAnglePowerExperimental(moduleStates[3]);
  }

  public String getModuleWantedTranslationVelocity(double velocityX, double velocityY, double angularVelocity) {
    double angle = getAngle();
    ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(velocityX * Constants.MAX_TRANS_METERS_PER_SEC, velocityY * Constants.MAX_TRANS_METERS_PER_SEC, angularVelocity * Constants.MAX_ANG_RAD_PER_SEC, Rotation2d.fromDegrees(angle));

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_TRANS_METERS_PER_SEC);
    return moduleStates[0].speedMetersPerSecond + " " + moduleStates[2].speedMetersPerSecond + " " + moduleStates[1].speedMetersPerSecond + " " + moduleStates[3].speedMetersPerSecond;
  }

  // Other
  public String getModuleVelocities() {
    return lfModule.getVelocity() + " " + lbModule.getVelocity() + " " + rfModule.getVelocity() + " " + rbModule.getVelocity();
  }

  public String getModuleTranslationPositions() {
    return lfModule.getTransPosition() + " " + lbModule.getTransPosition() + " " + rfModule.getTransPosition() + " " + rbModule.getTransPosition();
  }
  public void updateOdometry(){
    odometry.update(Rotation2d.fromDegrees(getAngle()), new SwerveModulePosition[]{lfModule.getModulePosition(),rfModule.getModulePosition(),lbModule.getModulePosition(),rbModule.getModulePosition()});
  }
  public String displayModulePositionDist(){
    double lfDist = lfModule.getModulePosition().distanceMeters;
    double rfDist = rfModule.getModulePosition().distanceMeters;
    double lbDist = lbModule.getModulePosition().distanceMeters;
    double rbDist = rbModule.getModulePosition().distanceMeters;
    return Double.toString(lfDist) + " " + Double.toString(rfDist) + " " + Double.toString(lbDist) + " " + Double.toString(rbDist);
  }
  public String displayModulePositionAng(){
    Rotation2d lfAng = lfModule.getModulePosition().angle;
    Rotation2d rfAng = rfModule.getModulePosition().angle;
    Rotation2d lbAng = lbModule.getModulePosition().angle;
    Rotation2d rbAng = rbModule.getModulePosition().angle;
    return Double.toString(lfAng.getDegrees()) + " " + Double.toString(rfAng.getDegrees()) + " " + Double.toString(lbAng.getDegrees()) + " " + Double.toString(rbAng.getDegrees());
  }
  public double getXPose(){
    return odometry.getPoseMeters().getX();
  }
  public double getYPose(){
    return odometry.getPoseMeters().getY();
  }
  public double getHeadingPose(){
    return odometry.getPoseMeters().getRotation().getDegrees();
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public String x(){
    String X = Double.toString(odometry.getPoseMeters().getX());
    return X;
  }
  public String y(){
    String Y = Double.toString(odometry.getPoseMeters().getY());
    return Y;
  }
  public String z(){
    String Z = Double.toString(odometry.getPoseMeters().getRotation().getDegrees());
    return Z;
  }
  public SwerveDriveKinematics gKinematics(){
    return kinematics;
  }
  public void resetOdometry(Pose2d var){
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),new SwerveModulePosition[]{lfModule.getModulePosition(),rfModule.getModulePosition(),lbModule.getModulePosition(),rbModule.getModulePosition()} , var);
  }
  public void setModuleStates(SwerveModuleState[]desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_TRANS_METERS_PER_SEC);
    lfModule.set(desiredStates[0], false);
    rfModule.set(desiredStates[1], false);
    lbModule.set(desiredStates[2], false);
    rbModule.set(desiredStates[3], false);
  }
}
