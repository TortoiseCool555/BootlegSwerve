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
  PIDController transPID;

  SwerveModuleState moduleState;
  double previousAngle;

  public NewSwerveModule(int trans, int rot, int rotEnc, double offset) {
    translation = new TalonFX(trans, "CANivoreA");
    rotation = new TalonFX(rot, "CANivoreA");
    translation.setNeutralMode(NeutralMode.Brake);
    rotation.setNeutralMode(NeutralMode.Brake);
    rotationEncoder = new CANCoder(rotEnc, "CANivoreA");

    rotPID = new PIDController(Constants.pRot, 0.027, 0);
    rotPID.enableContinuousInput(-180, 180);
    rotPID.setTolerance(0.3);
    rotPID.setIntegratorRange(-0.2, 0.2);

    transPID = new PIDController(0, 0, 0);
    transPID.enableContinuousInput(-Constants.MAX_TRANS_METERS_PER_SEC, Constants.MAX_TRANS_METERS_PER_SEC);
    transPID.setTolerance(10);
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
  
  public void set(SwerveModuleState wanted, double angle, boolean isStalled) {
    double additional = ExtraMath.clip(transPID.calculate(translation.getSelectedSensorVelocity(), wanted.speedMetersPerSecond), .2);
    translation.set(ControlMode.PercentOutput, additional + (wanted.speedMetersPerSecond / Constants.MAX_TRANS_METERS_PER_SEC));

   double wantedAngle = wanted.angle.getDegrees();
   if(isStalled) {
     //wantedAngle = previousAngle;
   }

   if(Math.abs(rotPID.getPositionError()) < 30) {
     rotPID.setI(.027);
   } else {
     rotPID.setI(0);
   }

    rotation.set(ControlMode.PercentOutput, ExtraMath.clip(rotPID.calculate(rotationEncoder.getAbsolutePosition(), wantedAngle), 1.0));
    previousAngle = wantedAngle;
  }
  
  public double getWantedAngle(SwerveModuleState wanted, double angle) {
    return SwerveModuleState.optimize(wanted, Rotation2d.fromDegrees(angle)).angle.getDegrees();
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
}
