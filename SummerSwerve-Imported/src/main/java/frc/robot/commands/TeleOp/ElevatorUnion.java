// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOp;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
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
  boolean shouldReset = true;
  boolean scoringMode = false;
  boolean cubeMode = true;
  Toggle adjustmentToggle = new Toggle(1);
  double sequenceNum = 1;
  double elColor;
  boolean isEncoders = true;

  String elString = "Stationary";

  public ElevatorUnion(Elevator elevator, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.controller = controller;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetElevator();
    wantedElevatorPos = 0;
    driverIntentExtend = 0;
    driverIntentArm = 0;
    armAdjustment = 0;
    extendAdjustment = 0;
    shouldReset = true;
    scoringMode = false;
    cubeMode = true;
    sequenceNum = 1;
    isEncoders = true;
    elevator.setArmPID(.3, 0, 0.00145);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double lowAngle = 236.5;
    // if(Math.abs(elevator.getPosition()) > 1000) {
    //   lowAngle = 250;
    // }
    // Set Elevator Position
    double controllerVal = -controller.getLeftY();
    if(Math.abs(controllerVal) < 0.1) {
      controllerVal = 0;
    }
    controllerVal = controllerVal < 0 ? controllerVal * 200 * 0.8 : controllerVal * 200;
    wantedElevatorPos += controllerVal;
    wantedElevatorPos = ExtraMath.clip(wantedElevatorPos, 3, 9500);

    // Driver adjustments
    if(driverIntentArm + armAdjustment > 236.5 && -controller.getRightY() < 0) {
      
    } else if(driverIntentArm + armAdjustment < 60 && -controller.getRightY() > 0) {
      
    } else {
      armAdjustment -= Math.abs(controller.getRightY()) < 0.1 ? 0 :  ExtraMath.exponential(-controller.getRightY(), 2.9, 0);
    }

    if(driverIntentExtend + extendAdjustment > 17.5 && controller.getLeftX() > 0) {
      
    } else if(driverIntentExtend + extendAdjustment < 0 && controller.getLeftX() < 0) {
      
    } else {
      extendAdjustment += Math.abs(controller.getLeftX()) < 0.1 ? 0 : controller.getLeftX();
    }
    
    // armAdjustment = ExtraMath.clip(armAdjustment, 62, 216);
   
    // Select Element Mode
    if(controller.getLeftBumperPressed()) {
      cubeMode = !cubeMode;
    }
    // Select Scoring Mode
    if(controller.getRightBumperPressed()){
      scoringMode = !scoringMode;
    }

    // Reset Mode
    if(controller.getBButtonPressed()) {
      shouldReset = !shouldReset;
    }

    // Select driver intent
    if(scoringMode) {
      if(cubeMode) { // SCoring Cube
        elColor = 0.89;
        if (elevator.getPosition() < -7000) {
          sequenceNum = 4;
          elString = "High Cube";
          driverIntentExtend = 15;
          driverIntentArm = 135;
        } else if (elevator.getPosition() < -1000) {
          sequenceNum = 3;
          elString = "Mid Cube";
          driverIntentExtend = 5;
          driverIntentArm = 135;
        } else {
          sequenceNum = 2;
          elString = "Low/Mid Cube";
          driverIntentExtend = 0;
          driverIntentArm = 135;
        }
      } // SCoring Cones
      else {
        elColor = 0.69;
        if(elevator.getPosition() < -7000) {
          sequenceNum = 7;
          elString = "High Cone";
          driverIntentExtend = 10;
          driverIntentArm = 265;
        } else if (elevator.getPosition() < -1000) {
          sequenceNum = 6;
          elString = "Mid Cone";
          driverIntentExtend = 5;
          driverIntentArm = 165;
        } else {
          sequenceNum = 5;
          elString = "Low Cone";
          driverIntentExtend = 0;
          driverIntentArm = 135;
        }
      }
    } 
    else{ // Grab mode
      elColor = 0.89;
      if(cubeMode) {
        if(elevator.getPosition() < -6000) {
          sequenceNum = 9;
          elString = "Alliance Station Cube";
          driverIntentExtend = 5;
          driverIntentArm = 135;
        } else {
          sequenceNum = 8;
          elString = "Intake Cube";
          driverIntentExtend = 0;
          driverIntentArm = 180;
        }
      } else {
        elColor = 0.69;
        if(elevator.getPosition() < -6000) {
          sequenceNum = 11;
          elString = "Alliance Station Cone";
          driverIntentExtend = 5;
          driverIntentArm = 135;
        } else {
          sequenceNum = 10;
          elString = "Intake Cone";
          driverIntentExtend = 0;
          driverIntentArm = 160;
        }
      }
    }

    // if(shouldReset) {
    //   sequenceNum = 1;
    //   elColor = 0.01;
    //   driverIntentArm = 75;
    //   driverIntentExtend = 0;
    // } 
    sequenceNum = 1;
    elColor = 0.01;
    driverIntentArm = 75;
    driverIntentExtend = 0;

    // if(adjustmentToggle.isToggled(sequenceNum)) {
    //   armAdjustment = 0;
    //   extendAdjustment = 0;
    // }

    if(controller.getXButtonPressed()) {
      isEncoders = true;
      wantedElevatorPos = 0;
      driverIntentExtend = 0;
      driverIntentArm = 75;
      extendAdjustment = 0;
      armAdjustment = 0;
      elevator.resetElevator();
    } else if(controller.getAButtonPressed()) {
      isEncoders = false;
    }

    if(isEncoders) {
      elevator.setPosition(wantedElevatorPos, true);
      elevator.setExtend(driverIntentExtend + extendAdjustment);
      elevator.setArmAngle(driverIntentArm + armAdjustment);
    } else {
      elevator.setPower(controller.getLeftY() * 0.6);
      elevator.setExtendPower(controller.getLeftX() * 0.3);
      elevator.setArmPower(-controller.getRightY() * 0.2);
    }

    if(controller.getRightTriggerAxis() > 0.1) {
      elevator.setIntake(-0.4);
    } else if(controller.getLeftTriggerAxis() > 0.1) {
      elevator.setIntake(0.65);
    } else {
      elevator.setIntake(0);
    }

    // elevator.setColor(elColor);
    if(Constants.scoringMode) {
      elevator.setColor(0.69);
    } else {
      elevator.setColor(0.89);
    }

    // Driver dashboard
    SmartDashboard.putBoolean("Scoring Mode", scoringMode);
    SmartDashboard.putBoolean("Cube Mode", cubeMode);
    SmartDashboard.putString("Elevator Prediction", elString);

    // Program Values
    SmartDashboard.putNumber("Elevator", -elevator.getPosition());
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
