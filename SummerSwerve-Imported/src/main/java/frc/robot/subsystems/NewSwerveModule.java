// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;

public class NewSwerveModule extends SubsystemBase {
  /** Creates a new NewSwerveModule. */
  TalonFX translation;
  TalonFX rotation;
  CANCoder rotationEncoder;

  PIDController rotPID;
  PIDController transPID;

  SwerveModuleState moduleState;
  double previousAngle;
  double offset;

  public NewSwerveModule(int trans, int rot, int rotEnc, double offset) {
    translation = new TalonFX(trans, "CANivoreA");
    translation.configNeutralDeadband(0.001);
    rotation = new TalonFX(rot, "CANivoreA");
    rotation.configNeutralDeadband(0.001);

    translation.setNeutralMode(NeutralMode.Coast);
    rotation.setNeutralMode(NeutralMode.Brake);
    rotationEncoder = new CANCoder(rotEnc, "CANivoreA");
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    rotation.configOpenloopRamp(0.04);

    rotPID = new PIDController(0.007, 0, 0.000000011);
    rotPID.enableContinuousInput(-180, 180);
    rotPID.setTolerance(Math.toRadians(0.1));

    transPID = new PIDController(0.00, 0, 0);
    transPID.enableContinuousInput(-Constants.MAX_TRANS_METERS_PER_SEC, Constants.MAX_TRANS_METERS_PER_SEC);
    this.offset = offset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initialize() {
    translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // translation.setSelectedSensorPosition(0);
    rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void resetTransEncoder() {
    translation.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getModuleState(double angle) {
    return new SwerveModuleState(translation.getSelectedSensorVelocity(), Rotation2d.fromDegrees(rotationEncoder.getPosition()));
  }
  public void set(SwerveModuleState wanted1, double angle, boolean isStalled) {
    // Test optimization
    SwerveModuleState wanted = SwerveModuleState.optimize(wanted1, new Rotation2d(Math.toRadians(rotationEncoder.getAbsolutePosition())));
    double additional = ExtraMath.clip(transPID.calculate(translation.getSelectedSensorVelocity(), wanted.speedMetersPerSecond), .2);
    translation.set(ControlMode.PercentOutput, additional + (wanted.speedMetersPerSecond / Constants.MAX_TRANS_METERS_PER_SEC));

   double wantedAngle = ExtraMath.mod(wanted.angle.getDegrees() + 180.0 + offset, 360.0) - 180.0;

    rotation.set(ControlMode.PercentOutput, ExtraMath.clip(rotPID.calculate(rotationEncoder.getAbsolutePosition(), wantedAngle), 1));
    previousAngle = wantedAngle;
  }
  
  public double getWantedAngle(SwerveModuleState wanted, double angle) {
    return SwerveModuleState.optimize(wanted, Rotation2d.fromDegrees(angle)).angle.getDegrees();
  }
  public double getAngleRot(){
     return rotationEncoder.getAbsolutePosition();
  }

  public double getVelocity() {
    return (translation.getSelectedSensorVelocity())/60 * Constants.WHEEL_DIAM * Constants.DRIVING_GEAR_RATIO;
  }

  public double getTransPosition() {
    return translation.getSelectedSensorPosition();
  }

  public double getPositionError() {
    return rotPID.getPositionError();
  }

  public  double getAngleErrorExperimental(SwerveModuleState wanted) {
    return wanted.speedMetersPerSecond;
  }

  public double getAnglePowerExperimental(SwerveModuleState wanted) {
    // double rotationDiff = rotPID.
    // return rotation.getSelectedSensorVelocity()/Constants.MAX_ROT_PER_SEC;
    return ExtraMath.clip(rotPID.calculate(rotationEncoder.getAbsolutePosition(), wanted.angle.getDegrees()), 1);
  }
  public SwerveModulePosition getModulePosition(){
    SwerveModulePosition var = new SwerveModulePosition(translation.getSelectedSensorPosition() * Constants.DRIVING_GEAR_RATIO / Constants.TICKS_PER_REVOLUTION * Constants.WHEEL_CIRCUM, Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition()));
    return var;
  }
  public void resetCanCoder(){
    rotationEncoder.setPosition(0);
  }

  public void setBrake() {
    translation.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    translation.setNeutralMode(NeutralMode.Coast);
  }
}
