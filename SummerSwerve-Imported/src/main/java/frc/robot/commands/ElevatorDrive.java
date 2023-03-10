// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorDrive extends CommandBase {
  Elevator elevator;
  XboxController controller;
  double pos = 0;
  double groundVal;
  double angle =62;
  double distExt = 0;
  boolean var = false;
  DecimalFormat df = new DecimalFormat("0.00");
  /** Creates a new ElevatorDrive. */
  public ElevatorDrive(Elevator elevator, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setBrake();
    elevator.resetElevator();
    elevator.setExBrake();
    elevator.startComp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundVal = Math.abs(controller.getLeftY()) < 0.1 ? 0 : -controller.getLeftY();
    angle += Math.abs(-controller.getRightY()) < 0.1 ? 0 : -controller.getRightY() * 1.5;
    distExt += Math.abs(controller.getLeftX()) < 0.1 ? 0 : controller.getLeftX();
    pos += groundVal * 200;
    if(pos < 0){
      pos = 0;
    }
    else if(pos > 11600){
      pos = 11600;
    }
    else{
      pos += groundVal;
    }
    if(angle > 270){
      angle = 270;
    }
    else if(angle < 62){
      angle = 62;
    }
    if(controller.getLeftBumper()){
      SmartDashboard.putString("A", "Pushed");
      var = false;
    }
    else if(controller.getRightBumper()){
      SmartDashboard.putString("A", "not");
      var = true;
    }
    if(distExt > 17.6){
      distExt = 17.6;
    }
    else if(distExt < 0){
      distExt = 0;
    }
    // elevator.setPower(pos);
    elevator.setExtend(distExt);
    elevator.setArmPower(angle);
    elevator.switchStates(var);
    if(controller.getLeftTriggerAxis() > 0.1) {
      elevator.setIntake(0.5);
    } else if(controller.getRightTriggerAxis()  > 0.1) {
      elevator.setIntake(-0.5);
    } else {
      elevator.setIntake(0);
    }
    SmartDashboard.putString("Elevator", elevator.positionString());
    SmartDashboard.putBoolean("Solenoid", elevator.getSolenoidState());
    SmartDashboard.putNumber("Left Pos: ", elevator.getLeftPos());
    SmartDashboard.putNumber("Right Pos: ", elevator.getRightPos());
    SmartDashboard.putString("Desired Position: ", df.format(pos));
    SmartDashboard.putNumber("Arm Power", elevator.setArm(angle));
    SmartDashboard.putNumber("Desired Arm Angle", angle);
    SmartDashboard.putNumber("Desired Extension", distExt);
    SmartDashboard.putNumber("Arm Angle", elevator.armAng());
    SmartDashboard.putNumber("Arm Ang Pow", elevator.setArm(angle));
    SmartDashboard.putNumber("Extension Distance", elevator.getExtDist());
    SmartDashboard.putNumber("Extension Power", elevator.setExtend(distExt));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.zeroPower();
    elevator.stopComp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
