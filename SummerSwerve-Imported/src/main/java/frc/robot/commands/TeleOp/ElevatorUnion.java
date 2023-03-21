// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOp;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Toggle;
import frc.robot.subsystems.Elevator;

public class ElevatorUnion extends CommandBase {
  /** Creates a new ElevatorUnion. */
  Elevator elevator;
  XboxController controller;

  // Stored numbers
  double wantedElevatorPos = 0;
  double driverIntentExtend = 0;
  double driverIntentArm = 0;
  double armAdjustment = 0;
  double extendAdjustment = 0;
  boolean shouldReset = false;
  boolean scoringMode = false;
  Toggle adjustmentToggle = new Toggle(1);
  double sequenceNum = 1;
  double elColor;

  public ElevatorUnion(Elevator elevator, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetElevator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set Elevator Position
    wantedElevatorPos += -controller.getLeftY();
    elevator.setPosition(wantedElevatorPos, true);

    // Driver adjustments
    extendAdjustment += controller.getLeftX();
    armAdjustment += -controller.getRightY();
   
    // Select Scoring Mode
    if(controller.getRightBumperPressed()){
      scoringMode = true;
    } else if(controller.getLeftBumperPressed()) {
     scoringMode = false;
    }

    // Reset Mode
    if(controller.getBButtonPressed()) {
      shouldReset = !shouldReset;
    }

    // Select driver intent
    if(shouldReset) {
      sequenceNum = 1;
      elColor = 0.01;
      driverIntentArm = 70;
      driverIntentExtend = 0;
    } else if(scoringMode) {
      elColor = 0.69;
      if (elevator.getPosition() > 6000) {
        sequenceNum = 4;
        driverIntentExtend = 11;
        driverIntentArm = 160;
      } else if (elevator.getPosition() > 3000) {
        sequenceNum = 3;
        driverIntentExtend = 5;
        driverIntentArm = 160;
      } else {
        sequenceNum = 2;
        driverIntentExtend = 0;
        driverIntentArm = 120;
      }
    } else{ // Grab mode
      elColor = 0.89;
      if (elevator.getPosition() > 6000) {
        sequenceNum = 7;
        driverIntentExtend = 5;
        driverIntentArm = 160;
      } else if(elevator.getPosition() > 3000) {
        sequenceNum = 6;
        driverIntentExtend = 0;
        driverIntentArm = 80;
      } else {
        sequenceNum = 5;
        driverIntentExtend = 0;
        driverIntentArm = 160;
      }
    }

    if(adjustmentToggle.isToggled(sequenceNum)) {
      armAdjustment = 0;
      extendAdjustment = 0;
    }
    
    elevator.setExtend(driverIntentExtend + extendAdjustment);
    elevator.setArmAngle(driverIntentArm + armAdjustment);

    if(controller.getLeftTriggerAxis() > 0.1 && shouldReset) {
      elevator.setIntake(0.9);
      elColor = 0.7;
    } if(controller.getLeftTriggerAxis() > 0.1) {
      elevator.setIntake(0.3);
      elColor = 0.6;
    } else if(controller.getRightTriggerAxis()  > 0.1) {
      elevator.setIntake(-0.3);
    } else {
      elevator.setIntake(0);
    }

    elevator.setColor(elColor);

    SmartDashboard.putBoolean("Mode", scoringMode);
    SmartDashboard.putNumber("Elevator", elevator.getPosition());
    SmartDashboard.putNumber("Arm Angle", elevator.getArmAngle());
    SmartDashboard.putNumber("Extension Distance", elevator.getExtDist());
    SmartDashboard.putNumber("Desired Position", wantedElevatorPos);
    SmartDashboard.putNumber("Desired  Arm Angle", driverIntentArm + armAdjustment);
    SmartDashboard.putNumber("Desired Extension", driverIntentExtend + extendAdjustment);
    SmartDashboard.putNumber("Position Sequence", sequenceNum);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
