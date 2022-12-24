// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
  PIDController rotPIDTester;

  SwerveModuleState moduleState;
  double previousAngle;

  public NewSwerveModule(int trans, int rot, int rotEnc) {
    translation = new TalonFX(trans);
    rotation = new TalonFX(rot);
    translation.setNeutralMode(NeutralMode.Brake);
    rotation.setNeutralMode(NeutralMode.Brake);
    rotationEncoder = new CANCoder(rotEnc);
    rotPID = new PIDController(2, 0, 0);
    rotPID.enableContinuousInput(-180, 180);
    rotPID.setTolerance(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initialize() {
    translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public SwerveModuleState getModuleState(double angle) {
    return new SwerveModuleState(translation.getSelectedSensorVelocity(), Rotation2d.fromDegrees(rotationEncoder.getPosition()));
  }
  /////////////////////////////////////////////////////////////////////////////////
  
  public void set(SwerveModuleState wanted, double angle, boolean isStalled) {
    //wanted = SwerveModuleState.optimize(wanted, Rotation2d.fromDegrees(angle));

    translation.set(ControlMode.PercentOutput, wanted.speedMetersPerSecond / Constants.MAX_TRANS_PER_SEC);
    
    // rotPID.setIntegratorRange(-.5, .5);
   // double pow = rotPID.calculate(rotationEncoder.getAbsolutePosition(), wanted.angle.getDegrees());

   double wantedAngle = wanted.angle.getDegrees();
   if(isStalled) {
     wantedAngle = previousAngle;
   }

    rotation.set(ControlMode.PercentOutput, (rotPID.calculate(rotationEncoder.getAbsolutePosition(), wantedAngle))/180);
    previousAngle = wantedAngle;
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  public double getWantedAngle(SwerveModuleState wanted, double angle) {
    return SwerveModuleState.optimize(wanted, Rotation2d.fromDegrees(angle)).angle.getDegrees();
  }

  public double getVelocity() {
    return translation.getSelectedSensorVelocity();
  }

  public double getTransPosition() {
    return translation.getSelectedSensorPosition();
  }

  public double getPositionError() {
    return rotPID.getPositionError();
  }

  public  double getAngleErrorExperimental(SwerveModuleState wanted) {
    return ExtraMath.simpleAngleError(rotationEncoder.getPosition(), wanted.angle.getDegrees());
  }

  public double getAnglePowerExperimental(SwerveModuleState wanted) {
    // double rotationDiff = rotPID.
    // return rotation.getSelectedSensorVelocity()/Constants.MAX_ROT_PER_SEC;
    return wanted.angle.getDegrees();
  }
}
