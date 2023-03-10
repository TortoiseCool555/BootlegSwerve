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
  double angle;
  double extend;
  public PrepElevator(Elevator elevator, int pos, double extend) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.pos = pos;
    this.extend = extend;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = elevator.armAng();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevator.setPower(pos);
    elevator.setArm(angle);
    elevator.setExtend(extend); 
    elevator.setIntake(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(pos - elevator.getPosition()) < 40;
    return false;
  }
}
