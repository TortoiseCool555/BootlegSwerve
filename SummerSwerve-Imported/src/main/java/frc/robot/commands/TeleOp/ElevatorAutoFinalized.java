// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.Toggle;
import frc.robot.subsystems.Elevator;

public class ElevatorAutoFinalized extends CommandBase {
  /** Creates a new ElevatorAutoFinalized. */
  Elevator elevator;
  XboxController controller;
  int sequenceNum = 0;
  Toggle POVToggle = new Toggle(0);
  Toggle sequenceToggle = new Toggle(0);
  double previousElevator = 0;
  Timer timer = new Timer();

  double adjustmentElevator = 0;
  double adjustmentExtend = 0;
  double adjustmentArm = 0;
  int heightSequence = 0;
  int previousHeightSequence = 0;
  boolean goingUp = true;
  // double storedElColor = 0;

  String heightString = "NONE";

  double driverIntentElevator = 0;
  double driverIntentExtend = 0;
  double driverIntentArm = 75;
  double subElevator = 0;
  double storedElColor = 0;
  double elColor = 0;
  public ElevatorAutoFinalized(Elevator elevator, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.controller = controller;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    adjustmentElevator = 0;
    adjustmentExtend = 0;
    adjustmentArm = 0;
    sequenceNum = 0;
    elevator.resetElevator();
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Driver adjustments
    subElevator = driverIntentElevator;
    double elStick = Math.abs(controller.getLeftY()) < 0.1 ? 0 : -controller.getLeftY();
    if(driverIntentElevator + adjustmentElevator > 9500 && elStick > 0) {
      
    } else if(driverIntentElevator + adjustmentElevator < 0 && elStick < 0) {
      
    } else {
      adjustmentElevator += elStick < 0 ? elStick * 200 * 0.8 : elStick * 200;
    }

    if(driverIntentExtend + adjustmentExtend > 17.5 && controller.getLeftX() > 0) {
      
    } else if(driverIntentExtend + adjustmentExtend < 0 && controller.getLeftX() < 0) {
      
    } else {
      adjustmentExtend += Math.abs(controller.getLeftX()) < 0.1 ? 0 : controller.getLeftX();
    }

    if(driverIntentArm + adjustmentArm > 236.5 && -controller.getRightY() < 0) {
      
    } else if(driverIntentArm + adjustmentArm < 60 && -controller.getRightY() > 0) {
      
    } else {
      adjustmentArm -= Math.abs(controller.getRightY()) < 0.1 ? 0 :  ExtraMath.exponential(-controller.getRightY(), 2.9, 0);
    }

    int currentPOV = controller.getPOV();
    boolean pov = POVToggle.isToggled(currentPOV);

    if(controller.getAButtonPressed()) {
      sequenceNum = 0;
      heightSequence = 0;
      driverIntentElevator = 3;
      driverIntentExtend = 0;
      driverIntentArm = 75;
      storedElColor = 0.89;
      heightString = "Score Cube Low";
    } else if(controller.getBButtonPressed()) {
      sequenceNum = 1;
      heightSequence = 1;
      driverIntentElevator = 2000;
      driverIntentExtend = 0;
      driverIntentArm = 90;
      storedElColor = 0.89;
      heightString = "Score Cube Mid";
    } else if(controller.getYButtonPressed()) {
      sequenceNum = 2;
      heightSequence = 2;
      driverIntentElevator = 9000;
      driverIntentExtend = 17;
      driverIntentArm = 120;
      storedElColor = 0.89;
      heightString = "Score Cube High";
    } else if(controller.getXButtonPressed()) {
      sequenceNum = 3;
      heightSequence = 0;
      driverIntentElevator = 3;
      driverIntentExtend = 0;
      driverIntentArm = 180;
      storedElColor = 0.89;
      heightString = "Collect Cube";
    } else if(controller.getRightBumperPressed()) {
      sequenceNum = 4;
      heightSequence = 2;
      driverIntentElevator = 9000;
      driverIntentExtend = 0;
      driverIntentArm = 180;
      storedElColor = 0.89;
      heightString = "Cube Alliance Station";
    } else if(pov && currentPOV == 180) {
      sequenceNum = 5;
      heightSequence = 0;
      driverIntentElevator = 3;
      driverIntentExtend = 0;
      driverIntentArm = 135;
      storedElColor = 0.69;
      heightString = "Score Cone Low";
    } else if(pov && currentPOV == 90) {
      sequenceNum = 6;
      heightSequence = 1;
      driverIntentElevator = 9000;
      driverIntentExtend = 3;
      driverIntentArm = 195;
      storedElColor = 0.69;
      heightString = "Score Cone Mid";
    } else if(pov && currentPOV == 0) {
      sequenceNum = 7;
      heightSequence = 2;
      driverIntentElevator = 9400;
      driverIntentExtend = 17;
      driverIntentArm = 165.5;
      storedElColor = 0.69;
      heightString = "Score Cone High";
    } else if(pov && currentPOV == 270) {
      sequenceNum = 8;
      heightSequence = 0;
      driverIntentElevator = 3;
      driverIntentExtend = 0;
      driverIntentArm = 227;
      storedElColor = 0.69;
      heightString = "Collect Cone";
    } else if(controller.getLeftBumperPressed()) {
      sequenceNum = 9;
      heightSequence = 2;
      driverIntentElevator = 9000;
      driverIntentExtend = 0;
      driverIntentArm = 180;
      storedElColor = 0.69;
      heightString = "Cone Alliance Station";
    }

    // if(sequenceNum == 0) {
      
    // } else if(sequenceNum == 1) {
      
    // } else if(sequenceNum == 2) {
      
    // } else if(sequenceNum == 3) {
      
    // } else if(sequenceNum == 4) {
      
    // } else if(sequenceNum == 5) {
      
    // } else if(sequenceNum == 6) {
      
    // } else if(sequenceNum == 7) {
      
    // } else if(sequenceNum == 8) {
      
    // } else if(sequenceNum == 9) {
      
    // }
    elColor = storedElColor;

    if(controller.getRightTriggerAxis() > 0.1) {
      elevator.setIntake(-0.4);
      if(timer.get() < 1) {
      } else if(Math.abs(elevator.getIntakeVelocity()) < 2000) {
        elColor = -0.11;
      }
    } else if(Math.abs(controller.getLeftTriggerAxis()) > 0.1) {
      elevator.setIntake(0.65);
      if(timer.get() < 1) {
      } else if(Math.abs(elevator.getIntakeVelocity()) < 3000) {
        elColor = -0.11;
      }
    } else {
      timer.reset();
      elevator.setIntake(0);
    }

    if(sequenceToggle.isToggled(sequenceNum)) {
      previousElevator = subElevator;
      adjustmentElevator = 0;
      adjustmentExtend = 0;
      adjustmentArm = 0;
    }

    if(heightSequence - previousHeightSequence > 0) {       //Lift arm
      goingUp = true;
    }
    else if(heightSequence - previousHeightSequence < 0) { // Lower Arm
      goingUp = false;
    }

    if(goingUp) {
      // elevator.setPosition(driverIntentElevator + adjustmentElevator, true);

      // if(Math.abs(-elevator.getPosition() - (driverIntentElevator + adjustmentElevator)) < 500) {
      //   elevator.setExtend(driverIntentExtend + adjustmentExtend);
      // }

      // if(Math.abs(elevator.getExtDist() - (driverIntentExtend + adjustmentExtend)) < 2) {
      //   elevator.setArmAngle(driverIntentArm + adjustmentArm);
      // }
      elevator.setPosition(driverIntentElevator + adjustmentElevator, true);
      elevator.setArmAngle(driverIntentArm + adjustmentArm);
      elevator.setExtendConstrainedScore((driverIntentExtend + adjustmentExtend) * Constants.EXTENSION_TO_METERS, driverIntentArm + adjustmentArm + 0.01);
    } else {
      elevator.setArmAngle(driverIntentArm + adjustmentArm);

      if(Math.abs(elevator.getArmAngle() - (driverIntentArm + adjustmentArm)) < 10) {
        SmartDashboard.putNumber("Ext Power", elevator.setExtend(driverIntentExtend + adjustmentExtend));
      }

      if(Math.abs(elevator.getExtDist() - (driverIntentExtend + adjustmentExtend)) < 5) {
        elevator.setPosition(driverIntentElevator + adjustmentElevator, true);
      } else {
        elevator.setPosition(previousElevator, true);
      }

    }
    Constants.elevatorHeight = -elevator.getPosition();
    elevator.setColor(elColor);


    SmartDashboard.putNumber("Intake Veloc", elevator.getIntakeVelocity());
    SmartDashboard.putString("Elevator Setting", heightString);
    SmartDashboard.putBoolean("Going Up", goingUp);
    SmartDashboard.putNumber("POV", currentPOV);
    SmartDashboard.putNumber("El Height", -elevator.getPosition());
    SmartDashboard.putNumber("Predicted Max Ext", elevator.getExtPredicted(driverIntentExtend + adjustmentExtend));
    SmartDashboard.putNumber("Ext Wanted", driverIntentExtend + adjustmentExtend);
    SmartDashboard.putNumber("Ext Predicted Max", elevator.getExtPredicted(driverIntentExtend + adjustmentExtend));
    SmartDashboard.putNumber("Ext Pos", elevator.getExtDist());
    SmartDashboard.putNumber("Elevator Error", driverIntentElevator + adjustmentElevator + elevator.getPosition());
    SmartDashboard.putNumber("Angle Error", driverIntentArm + adjustmentArm - elevator.getArmAngle());
    
    previousHeightSequence = heightSequence;
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
