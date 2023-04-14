// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOp;

import java.text.DecimalFormat;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.Toggle;
import frc.robot.subsystems.Elevator;

public class ElevatorAutomized extends CommandBase {
  Elevator elevator;
  XboxController controller;
  double elevatorVal;
  double angle =62;
  double distExt = 0;
  boolean state = false;
  DecimalFormat df = new DecimalFormat("0.00");
  int heightSequence = 1;
  int previousHeighSequence = 1;
  Toggle POVToggle = new Toggle(-1);
  boolean goingUp = false;
  double previousElevator;
  double previousExtension;
  double previousAngle;
  double additionalAngle;
  /** Creates a new ElevatorDrive. */
  public ElevatorAutomized(Elevator elevator, XboxController controller) {
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
    elevator.setColor(0.69);
    additionalAngle = 0;
    angle = 80;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double subPreviousElevator = elevatorVal;
    double subPreviousExtension = distExt;
    int currentPOV = controller.getPOV();
    elevatorVal = Math.abs(controller.getLeftY()) < 0.1 ? 0 : -controller.getLeftY();
    additionalAngle += Math.abs(-controller.getRightY()) < 0.1 ? 0 : -controller.getRightY() * 1.5;
    distExt += Math.abs(controller.getLeftX()) < 0.1 ? 0 : controller.getLeftX();

    angle = ExtraMath.clip(angle, 62, 270);
    distExt = ExtraMath.clip(distExt, 0, 17.6);


    // Intake
    if(controller.getLeftTriggerAxis() > 0.1) {
      elevator.setIntake(0.3);
    } else if(controller.getRightTriggerAxis()  > 0.1) {
      elevator.setIntake(-0.3);
    } else {
      elevator.setIntake(0);
    }

    boolean pov = POVToggle.isToggled(currentPOV);
    if(pov && currentPOV == 0) {
      heightSequence ++;
    } else if(pov && currentPOV == 180) {
      heightSequence --;
    }

    if(Constants.scoringMode) {
      heightSequence = ExtraMath.loopNum(heightSequence, 3);
      elevator.setColor(0.5);
      if(heightSequence == 1) {
        elevatorVal = 40;
        distExt = 0;
        angle = 77;
      }else if(heightSequence == 2) {
        elevatorVal = 4000;
        distExt = 9;
        angle = 145;
      } else {
        elevatorVal = 11000;
        distExt = 14;
        angle = 145;
      }
    } else {
      heightSequence = ExtraMath.loopNum(heightSequence, 3);
      if(heightSequence == 1) {
        elevatorVal = 40;
        distExt = 0;
        elevator.setColor(0.69);
        angle = 80;
      } else if(heightSequence == 2) {
        elevatorVal = 40;
        distExt = 0;
        if(controller.getLeftBumper()) {
          angle = 175;
          elevator.setColor(0.89);
        } else if(controller.getRightBumper()) {
          angle = 195;
          elevator.setColor(0.69);
        }
      } else {
        elevatorVal = 11000;
        distExt = 0;
        angle = 168;
        elevator.setColor(0.69);
      }
    }

    if(heightSequence - previousHeighSequence > 0) {       //Lift arm
      goingUp = true;
      previousElevator = subPreviousElevator;
      previousExtension = subPreviousExtension;
      additionalAngle = 0;
    } 
    else if(heightSequence - previousHeighSequence < 0) { // Lower Arm
      goingUp = false;
      previousElevator = subPreviousElevator;
      previousExtension = subPreviousExtension;
      additionalAngle = 0;
    }

    if(goingUp) {
      elevator.setPosition(elevatorVal, true);

      if(Math.abs(-elevator.getPosition() - elevatorVal) < 100) {
        SmartDashboard.putString("Here", previousExtension + ", " + distExt);
        elevator.setExtend(distExt);
      }

      if(Math.abs(elevator.getExtDist() - distExt) < 2) {
        elevator.setArmAngle(angle + additionalAngle);
      }
    } else {
      elevator.setArmAngle(angle + additionalAngle);

      if(Math.abs(elevator.getArmAngle() - angle) < 5) {
        elevator.setExtend(distExt);
      }

      if(Math.abs(elevator.getExtDist() - distExt) < 1) {
        elevator.setPosition(elevatorVal, true);
      } else {
        elevator.setPosition(previousElevator, true);
      }
    }
    

    

    SmartDashboard.putNumber("POV", currentPOV);
    SmartDashboard.putNumber("Elevator Sequence", heightSequence);
    SmartDashboard.putBoolean("Going Up", goingUp);
    SmartDashboard.putNumber("Elevator Error", -elevator.getPosition() - elevatorVal);
    SmartDashboard.putNumber("Elevator Power", elevator.getPower(elevatorVal, true));
    SmartDashboard.putNumber("Extension Error", elevator.getExtDist() - distExt);
    SmartDashboard.putNumber("Angle Error", elevator.getArmAngle() - angle);
    SmartDashboard.putString("Elevator", elevator.positionString());
    // SmartDashboard.putBoolean("Solenoid", elevator.getSolenoidState());
    SmartDashboard.putNumber("Left Pos: ", elevator.getLeftPos());
    SmartDashboard.putNumber("Right Pos: ", elevator.getRightPos());
    SmartDashboard.putString("Desired Position: ", df.format(elevatorVal));
    SmartDashboard.putNumber("Arm Power", elevator.getArmPower(angle));
    SmartDashboard.putNumber("Desired Arm Angle", angle);
    SmartDashboard.putNumber("Desired Extension", distExt);
    SmartDashboard.putNumber("Arm Angle", elevator.getArmAngle());
    SmartDashboard.putNumber("Arm Ang Pow", elevator.getArmPower(angle));
    SmartDashboard.putNumber("Extension Distance", elevator.getExtDist());
    SmartDashboard.putNumber("Extension Power", elevator.getExtDist());
    previousHeighSequence = heightSequence;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorOff();
    // elevator.stopComp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
