// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOp;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
      elevator.setColor(0.01);
      driverIntentArm = 70;
      driverIntentExtend = 0;
    }
    else if(scoringMode) {
      elevator.setColor(0.69);
      if (elevator.getPosition() > 6000) {
        driverIntentExtend = 11;
        driverIntentArm = 160;
      } else if (elevator.getPosition() > 3000) {
        driverIntentExtend = 5;
        driverIntentArm = 160;
      } else {
        driverIntentExtend = 0;
        driverIntentArm = 120;
      }
    }
    else{ // Grab mode
      elevator.setColor(0.89);
      if (elevator.getPosition() > 6000) {
        driverIntentExtend = 5;
        driverIntentArm = 160;
      } else if(elevator.getPosition() > 3000) {
        driverIntentExtend = 0;
        driverIntentArm = 80;
      } else {
        driverIntentExtend = 0;
        driverIntentArm = 160;
      }
    }
    
    elevator.setExtend(driverIntentExtend + extendAdjustment);
    elevator.setArmAngle(driverIntentArm + armAdjustment);

    if(controller.getLeftTriggerAxis() > 0.1 && shouldReset) {
      elevator.setIntake(0.9);
    } if(controller.getLeftTriggerAxis() > 0.1) {
      elevator.setIntake(0.3);
    } else if(controller.getRightTriggerAxis()  > 0.1) {
      elevator.setIntake(-0.3);
    } else {
      elevator.setIntake(0);
    }
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
