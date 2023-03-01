// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorDrive;

public class Elevator extends SubsystemBase {
  private CANSparkMax LS = new CANSparkMax(16, MotorType.kBrushless);
  private CANSparkMax RS = new CANSparkMax(15,MotorType.kBrushless);
  private CANSparkMax ex = new CANSparkMax(17,MotorType.kBrushless);
  private RelativeEncoder LSEnc = LS.getEncoder();
  private RelativeEncoder RSEnc = RS.getEncoder();
  private DecimalFormat df = new DecimalFormat("0.00");

  XboxController controller;

  /** Creates a new Elevator. */
  public Elevator(XboxController controller){
    this.controller= controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new ElevatorDrive(this, controller));
  }
  public void setPower (double power){
    LS.set(power);
    RS.set(power);
  }
  public void setBrake(){
    LS.setIdleMode(IdleMode.kBrake);
    RS.setIdleMode(IdleMode.kBrake);

  }
  public void setCoast(){
    LS.setIdleMode(IdleMode.kCoast);
    RS.setIdleMode(IdleMode.kCoast);
  }
  
  public void setExPower(double power){
   ex.set(power);
  }
  public void setExBrake(){
    ex.setIdleMode(IdleMode.kBrake);

  }
  public String positionString(){
    return "Left: " + df.format(LSEnc.getPosition()) + " \n " + "Right: " + df.format(RSEnc.getPosition()); 
  }
  public void resetElevator(){
    LSEnc.setPosition(0);
    RSEnc.setPosition(0);
  }

}
