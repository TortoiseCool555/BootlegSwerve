// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.commands.TeleOp.ElevatorAutoFinalized;
import frc.robot.commands.TeleOp.ElevatorDrive;
import frc.robot.commands.TeleOp.ElevatorUnion;

public class Elevator extends SubsystemBase {
  // private Compressor compressor = new Compressor(24, PneumaticsModuleType.REVPH);
  // private Solenoid solenoid = new Solenoid(24, PneumaticsModuleType.REVPH, 9);
  private Spark color = new Spark(0);

  private CANSparkMax LS = new CANSparkMax(15, MotorType.kBrushless);
  private CANSparkMax RS = new CANSparkMax(16,MotorType.kBrushless);
  private CANSparkMax ex = new CANSparkMax(18,MotorType.kBrushless);
  private CANSparkMax arm1 = new CANSparkMax(19, MotorType.kBrushless);
  private CANSparkMax arm2 = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax intake1 = new CANSparkMax(21, MotorType.kBrushless);
  // private CANSparkMax intake2 = new CANSparkMax(22, MotorType.kBrushless);

  private RelativeEncoder LSEnc = LS.getEncoder();
  private RelativeEncoder RSEnc = RS.getEncoder();
  private RelativeEncoder EXEnc = ex.getEncoder();
  private RelativeEncoder intakEncoder = intake1.getEncoder();
  private Encoder liftEnc = new Encoder(0, 1);
  private Encoder armEnc = new Encoder(2,3);
  PIDController armController = new PIDController(.33, 0, 0.02); // 4, 0, 0.0005
  ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.048, .1); // 0.81, 0.3
  //private DecimalFormat df = new DecimalFormat("0.00");

  XboxController controller;

  /** Creates a new Elevator. */
  public Elevator(XboxController controller){
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new ElevatorAutoFinalized(this, controller));
  }

  // Init
  public void resetElevator(){
    LSEnc.setPosition(0);
    RSEnc.setPosition(0);
    liftEnc.reset();
    armEnc.reset();
    EXEnc.setPosition(0);
    setBrake();
    setExBrake();
    setSmartCurrentLimit();
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

  public void setSmartCurrentLimit() {
    LS.setSmartCurrentLimit(40);
    RS.setSmartCurrentLimit(40);
  }

  // Elevator
  public void setPosition(double pos, boolean slow){
    double high = 0.6;
    if(-pos - getPosition() > 0 && slow) {
      high = 0.25;
    }
    double Lpower = ExtraMath.clip(((-pos - getPosition())*0.00038) - 0.09, high);
    double Rpower = ExtraMath.clip(((-pos - getPosition())*0.00038) - 0.09, high);
    LS.set(Lpower);
    RS.set(Rpower);
  }

  public void setPower(double power) {
    LS.set(power);
    RS.set(power);
  }
  public double getPower(double pos, boolean slow) {
    double high = 0.6;
    if(-pos - getPosition() > 0 && slow) {
      high = 0.25;
    }
    return ExtraMath.clip(((-pos - getPosition())*0.00038)-0.09, high);
  }
  public void elevatorOff(){
    LS.set(0);
    RS.set(0);
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
  public String positionString(){
    return "Elevator Position: " + liftEnc.getDistance(); 
  }
  
  // Arm
  public void setArmAngle(double angle) {
    double pow = getArmPower(angle);
    arm1.set(pow);
    arm2.set(-pow);
  }
  public double getArmAngle(){
    double ticksFixed = ((-armEnc.getRaw()) + (60 * Constants.through_bore_TPR / 360)) % Constants.through_bore_TPR;
    return Math.toDegrees(ticksFixed * (2 * Math.PI/Constants.through_bore_TPR));
  }
  public double getArmPower(double angle){
    double ang = Math.toRadians(getArmAngle());
    double pow = armController.calculate(ang, Math.toRadians(angle));
    pow += armFeedforward.calculate(Math.toRadians(angle), 0);
    // double pow = Math.copySign(Math.pow(angle - getArmAngle(), 2)*0.008, angle - getArmAngle());
    // pow = Math.abs(pow) > 0.2 ? Math.copySign(0.2,pow) : pow;
    // double added = angle < 80 ? 0.008 : -0.09;
    return ExtraMath.clip(pow, 0.35);
  }

  public void setArmPower(double power) {
    arm1.set(power);
    arm2.set(-power);
  }
  
  // Extend
  public double setExtend(double pos){
    double power = ExtraMath.clip((pos - getExtDist()) * 0.14, 0.7);
    ex.set(power);
    return power;
  }
  public double setExtendConstrainedScore(double extend, double inputAng){
    double elPos = (-getPosition() * Constants.ELEVATOR_TO_METERS) + Constants.EL_HEIGHT_OFF_GROUND;
    double angleDeg = inputAng;
    double angleRad = Math.toRadians(inputAng);
    double wantedDist;
    double obstacleHeight;
    // Wanted Distance
    if(angleDeg > 180) {
      if(elPos + (Constants.ARM_LENGTH * Math.sin(3.14 - angleRad)) > Constants.HEIGHT_FOR_SCORE) {
        wantedDist = Constants.DIST_FROM_SCORE + Constants.HEIGHT_FOR_SCORE;
      } else {
        wantedDist = Constants.DIST_FROM_SCORE;
      }
    } else if(elPos > Constants.HEIGHT_FOR_SCORE) {
      wantedDist = Constants.DIST_FROM_SCORE + Constants.DIST_FOR_CUBE;
    } else {
      wantedDist = Constants.DIST_FROM_SCORE;
    }

    // Current Obstacle Height
    if(elPos > Constants.HEIGHT_FOR_SCORE) {
      obstacleHeight = Constants.HEIGHT_FOR_SCORE2;
    } else {
      obstacleHeight = Constants.HEIGHT_FOR_SCORE;
    }

    double predictedMax = (-(obstacleHeight - elPos) / Math.tan(3.14 - angleRad)) + wantedDist;
    double finalizedMax;
    if(angleDeg <= 90) {
      finalizedMax = wantedDist;
    } else if(angleDeg > 180 || predictedMax - (Constants.ARM_LENGTH * Math.cos(angleRad)) < wantedDist) {
      finalizedMax = wantedDist + (Constants.ARM_LENGTH * Math.cos(angleRad));
    } else {
      finalizedMax = predictedMax;
    }

    double wantedExtension = extend < finalizedMax ? extend : finalizedMax - 0.3;

    double power = ExtraMath.clip((wantedExtension - (getExtDist() * Constants.EXTENSION_TO_METERS)) * 4, 0.5);
    ex.set(power);
    return power;
  }

  public double getExtPredicted(double extend) {
    double elPos = (-getPosition() * Constants.ELEVATOR_TO_METERS) + Constants.EL_HEIGHT_OFF_GROUND;
    double angleDeg = getArmAngle();
    double angleRad = Math.toRadians(angleDeg);
    double wantedDist;
    double obstacleHeight;
    // Wanted Distance
    if(angleDeg > 180) {
      if(elPos + (Constants.ARM_LENGTH * Math.sin(3.14 - angleRad)) > Constants.HEIGHT_FOR_SCORE) {
        wantedDist = Constants.DIST_FROM_SCORE + Constants.HEIGHT_FOR_SCORE;
      } else {
        wantedDist = Constants.DIST_FROM_SCORE;
      }
    } else if(elPos > Constants.HEIGHT_FOR_SCORE) {
      wantedDist = Constants.DIST_FROM_SCORE + Constants.HEIGHT_FOR_SCORE;
    } else {
      wantedDist = Constants.DIST_FROM_SCORE;
    }

    // Current Obstacle Height
    if(elPos > Constants.HEIGHT_FOR_SCORE) {
      obstacleHeight = Constants.HEIGHT_FOR_SCORE2;
    } else {
      obstacleHeight = Constants.HEIGHT_FOR_SCORE;
    }

    double predictedMax = ((obstacleHeight - elPos) / Math.tan(3.14 - angleRad)) + wantedDist;
    double finalizedMax;
    if(angleDeg == 90) {
      finalizedMax = wantedDist;
    } else if(angleDeg > 180 || predictedMax - (Constants.ARM_LENGTH * Math.cos(angleRad)) < wantedDist) {
      finalizedMax = wantedDist + (Constants.ARM_LENGTH * Math.cos(angleRad));
    } else {
      finalizedMax = predictedMax;
    }
    return predictedMax;
  }
  public double getExtDist(){
    return EXEnc.getPosition();
  }

  public void setExtendPower(double power) {
    ex.set(power);
  }

  // Intake
  public void setIntake(double power){
    intake1.set(-power);
    // intake2.set(power);
  }

  // Compressor
  public void startComp(){
    // compressor.enableAnalog(60, 120);
  }
  public void stopComp(){
    // compressor.disable();
  }
  public void setState(boolean var){
    // solenoid.set(var);
  }
  public void switchState(){
    // solenoid.toggle();
  }
  public boolean getSolenoidState() {
    // return solenoid.get();
    return false;
  }

  // Color
  public void setColor(double colorNum) {
    color.set(colorNum);
  }

  public void setArmPID(double p, double i, double d) {
    armController.setP(p);
    armController.setI(i);
    armController.setD(d);
  }

  public double getPositionError() {
    return armController.getPositionError();
  }

  public double getDerivativeError(){
    return armController.getVelocityError();
  }

  public double getIntakeVelocity() {
    return intakEncoder.getVelocity();
  }

  // public void setArmFeedForward(double g, double k) {
  //   armFeedforward.
  // }
}
