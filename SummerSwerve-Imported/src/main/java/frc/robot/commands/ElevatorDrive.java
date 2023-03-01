// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorDrive extends CommandBase {
  Elevator elevator;
  XboxController controller;
  double pos = 0;
  double groundVal;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundVal = Math.abs(controller.getRightY()) < 0.1 ? 0 : -controller.getRightY();
    pos += groundVal;
    double lift = controller.getLeftY() * 1;
    elevator.setPower(lift);
    SmartDashboard.putString("Elevator", elevator.positionString());
    SmartDashboard.putString("Desired Position: ", df.format(pos));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
