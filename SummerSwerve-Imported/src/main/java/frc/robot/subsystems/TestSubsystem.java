// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TestCommand;

public class TestSubsystem extends SubsystemBase {
  /** Creates a new TestSubsystem. */
  XboxController controller;
  CANSparkMax test1 = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax test2 = new CANSparkMax(14, MotorType.kBrushless);
  public TestSubsystem(XboxController controller) {
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new TestCommand(controller, this));
  }
  public void init(){
    test1.setIdleMode(IdleMode.kBrake);
    test2.setIdleMode(IdleMode.kBrake);
  }
  public void setMotor(double val, boolean var){
    if(var){
      test1.set(val);
      test2.set(-val);
    }
    else{
      test1.set(-val);
      test2.set(val);
    }
  }
}
