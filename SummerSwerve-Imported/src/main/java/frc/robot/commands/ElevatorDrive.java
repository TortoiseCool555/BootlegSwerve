// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.subsystems.Elevator;

public class ElevatorDrive extends CommandBase {
  Elevator elevator;
  XboxController controller;
  double pos = 0;
  double elevatorVal;
  double angle =62;
  double distExt = 0;
  boolean state = false;
  DecimalFormat df = new DecimalFormat("0.00");
  int heightSequence = 0;
  int previousPOV = -1;
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
    elevator.setSmartCurrentLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int currentPOV = controller.getPOV();
    elevatorVal = Math.abs(controller.getLeftY()) < 0.1 ? 0 : -controller.getLeftY();
    angle -= Math.abs(-controller.getRightY()) < 0.1 ? 0 : -controller.getRightY() * 1.65;
    distExt += Math.abs(controller.getLeftX()) < 0.1 ? 0 : controller.getLeftX();

    pos = ExtraMath.clip(pos + (elevatorVal * 200), 0, 11600);
    angle = ExtraMath.clip(angle, 62, 270);
    distExt = ExtraMath.clip(distExt, 0, 17.6);

    if(controller.getLeftBumper()){
      state = false;
    }
    else if(controller.getRightBumper()){
      state = true;
    }

    if(currentPOV != previousPOV && currentPOV == 1) {
      heightSequence ++;
    } else if(currentPOV != previousPOV && currentPOV == 3) {
      heightSequence --;
    }
    heightSequence = ExtraMath.loopNum(heightSequence, 3);

    // Set elevator, then check extend, then check angle
    if(Constants.scoringMode) {
      
    } else {

    }

    Constants.elevatorHeight = elevator.getPosition();

    elevator.setPosition(pos, false);
    elevator.setExtend(distExt);
    elevator.setArmAngle(angle);
    elevator.setState(state);

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
    SmartDashboard.putNumber("Arm Power", elevator.getArmPower(angle));
    SmartDashboard.putNumber("Desired Arm Angle", angle);
    SmartDashboard.putNumber("Desired Extension", distExt);
    SmartDashboard.putNumber("Arm Angle", elevator.getArmAngle());
    SmartDashboard.putNumber("Arm Ang Pow", elevator.getArmPower(angle));
    SmartDashboard.putNumber("Extension Distance", elevator.getExtDist());
    SmartDashboard.putNumber("Extension Power", elevator.setExtend(distExt));
    previousPOV = currentPOV;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorOff();
    elevator.stopComp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
