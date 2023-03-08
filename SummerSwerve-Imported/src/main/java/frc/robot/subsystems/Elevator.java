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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.commands.ElevatorDrive;

public class Elevator extends SubsystemBase {
  private Compressor compressor = new Compressor(24, PneumaticsModuleType.REVPH);
  private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

  private CANSparkMax LS = new CANSparkMax(16, MotorType.kBrushless);
  private CANSparkMax RS = new CANSparkMax(15,MotorType.kBrushless);
  private CANSparkMax ex = new CANSparkMax(18,MotorType.kBrushless);
  private CANSparkMax arm1 = new CANSparkMax(19, MotorType.kBrushless);
  private CANSparkMax arm2 = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax intake1 = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax intake2 = new CANSparkMax(22, MotorType.kBrushless);

  private RelativeEncoder LSEnc = LS.getEncoder();
  private RelativeEncoder RSEnc = RS.getEncoder();
  private RelativeEncoder EXEnc = ex.getEncoder();
  private Encoder liftEnc = new Encoder(0, 1);
  private Encoder armEnc = new Encoder(2,3);
  private DecimalFormat df = new DecimalFormat("0.00");

  XboxController controller;

  /** Creates a new Elevator. */
  public Elevator(XboxController controller){
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new ElevatorDrive(this, controller));
  }
  public void setPower(double pos){
    double Lpower = ExtraMath.clip((-pos - getPosition())/4500.0, 0.5);
    double Rpower = ExtraMath.clip((-pos - getPosition())/4500.0, 0.5);
    LS.set(Lpower);
    RS.set(Rpower);
  }
  public void zeroPower(){
    LS.set(0);
    RS.set(0);
  }
  public void setBrake(){
    LS.setIdleMode(IdleMode.kBrake);
    RS.setIdleMode(IdleMode.kBrake);

  }
  public void setCoast(){
    LS.setIdleMode(IdleMode.kCoast);
    RS.setIdleMode(IdleMode.kCoast);
  }
  public void setExBrake(){
    ex.setIdleMode(IdleMode.kBrake);

  }
  public String positionString(){
    return "Elevator Position: " + liftEnc.getDistance(); 
  }
  public void resetElevator(){
    LSEnc.setPosition(0);
    RSEnc.setPosition(0);
    liftEnc.reset();
    armEnc.reset();
    EXEnc.setPosition(0);
  }
  public double getPosition(){
    return liftEnc.getDistance();
  }
  public double getLeftPos(){
    return LSEnc.getPosition();
  }
  public double getRightPos(){
    return RSEnc.getPosition();
  }
  public void setExt(double val){
    ex.set(val);
  }
  public double setArm(double angle){
    double pow = (angle - armAng())*0.005;
    pow = Math.abs(pow) > 0.6 ? Math.copySign(0.6,pow) : pow;
    return pow;
  }
  public void setArmPower(double angle) {
    double pow = setArm(angle);
    // pow = Math.abs(pow) > 0.8 ? Math.copySign(0.8,pow) : pow;
    arm1.set(-pow);
    arm2.set(pow);
  }
  public double armAng(){
    double ticksFixed = ((armEnc.getRaw() / 4.0 ) + (50 * Constants.through_bore_TPR / 360)) % Constants.through_bore_TPR;
    return Math.toDegrees(ticksFixed * (2 * Math.PI/Constants.through_bore_TPR));
  }
  public double getExtDist(){
    return EXEnc.getPosition();
  }
  public double setExtend(double pos){
    double power = (pos - getExtDist())/17.6;
    ex.set(power);
    return power;
  }
  public void setIntake(double power){
    intake1.set(-power);
    intake2.set(power);
  }
  public void startComp(){
    compressor.enableAnalog(80, 120);
  }
  public void stopComp(){
    compressor.disable();
  }
  public void switchStates(boolean var){
    solenoid.set(var);
  }
}
