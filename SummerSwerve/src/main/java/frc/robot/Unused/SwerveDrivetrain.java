// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Unused;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new Swerve. */
  private TalonFX module1Rotate = new TalonFX(2);
  private CANCoder moudle1RotEncoder = new CANCoder(1);
  private TalonFX module1Translate = new TalonFX(3);

  private TalonFX module2Rotate = new TalonFX(5);
  private CANCoder moudle2RotEncoder = new CANCoder(4);
  private TalonFX module2Translate = new TalonFX(6);

  private TalonFX module3Rotate = new TalonFX(8);
  private CANCoder moudle3RotEncoder = new CANCoder(7);
  private TalonFX module3Translate = new TalonFX(9);

  private TalonFX module4Rotate = new TalonFX(11);
  private CANCoder moudle4RotEncoder = new CANCoder(10);
  private TalonFX module4Translate = new TalonFX(12);

  private SwerveModule module1 = new SwerveModule(module1Rotate, module1Translate, moudle1RotEncoder, 1);
  private SwerveModule module2 = new SwerveModule(module2Rotate, module2Translate, moudle2RotEncoder, 2);
  private SwerveModule module3 = new SwerveModule(module3Rotate, module3Translate, moudle3RotEncoder, 3);
  private SwerveModule module4 = new SwerveModule(module4Rotate, module4Translate, moudle4RotEncoder, 4);

  private Pigeon2 gyro = new Pigeon2(13);

  Translation2d module1Location = new Translation2d(1, 1);
  Translation2d module2Location = new Translation2d(1, 1);
  Translation2d module3Location = new Translation2d(1, 1);
  Translation2d module4Location = new Translation2d(1, 1);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(module1Location, module2Location, module3Location, module4Location);
  ChassisSpeeds robotSpeeds = new ChassisSpeeds(0, 0, 0);
  
  double robotAngle = 0;

  XboxController controller;

  public SwerveDrivetrain(XboxController controller) {
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new FieldCentricSwerve(this, controller));
  }

  public void initGyro() {

  }

  public void setMotors(double translationAngle, double translationPercent, double turnPercent) {
    // double angle = getAngle();
    double angle = 0;
    module1.drive(translationAngle, translationPercent, turnPercent, angle);
    module2.drive(translationAngle, translationPercent, turnPercent, angle);
    module3.drive(translationAngle, translationPercent, turnPercent, angle);
    module4.drive(translationAngle, translationPercent, turnPercent, angle);
  }

  public void setRotMotors(double power) {
    module1Rotate.set(ControlMode.PercentOutput, -power);
    module2Rotate.set(ControlMode.PercentOutput, -power);
    module3Rotate.set(ControlMode.PercentOutput, -power);
    module4Rotate.set(ControlMode.PercentOutput, -power);

  }

  public double getModule1Deg() {
    return module1.getModuleAngle();
  }
  public double getModule2Deg() {
    return module2.getModuleAngle();
  }
  public double getModule3Deg() {
    return module3.getModuleAngle();
  }
  public double getModule4Deg() {
    return module4.getModuleAngle();
  }

  public double getAngle() {
    return gyro.getYaw();
  }
  
  public void setRobotSpeed(ChassisSpeeds speeds) {
    robotSpeeds = speeds;
  }

  public void stopMotors() {
    module1Rotate.set(ControlMode.PercentOutput, 0);
    module2Rotate.set(ControlMode.PercentOutput, 0);
    module3Rotate.set(ControlMode.PercentOutput, 0);
    module4Rotate.set(ControlMode.PercentOutput, 0);
    // module1Translate.set(ControlMode.PercentOutput, 0);
    // module2Translate.set(ControlMode.PercentOutput, 0);
    // module3Translate.set(ControlMode.PercentOutput, 0);
    // module4Translate.set(ControlMode.PercentOutput, 0);
  }
}
