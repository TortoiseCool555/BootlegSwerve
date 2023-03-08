// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class PrepElevator extends CommandBase {
  /** Creates a new PrepArm. */
  Elevator elevator;
  int pos;
  public PrepElevator(Elevator elevator, int pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double height = 0; // Maximum
    if(pos == 0) {
      height = 0;
    } else if(pos == 1) {
      height = 2;
    }

    elevator.setPower(height);
    elevator.setArm(50);
    elevator.setExtend(0); 
    elevator.setIntake(0.2);
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
