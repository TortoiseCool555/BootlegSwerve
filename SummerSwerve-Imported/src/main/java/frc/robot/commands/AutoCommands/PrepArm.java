// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class PrepArm extends CommandBase {
  /** Creates a new PrepArm. */
  Elevator elevator;
  double angle;
  double height;
  double extend;
  public PrepArm(Elevator elevator, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    height = elevator.getPosition();
    extend = elevator.getExtDist();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevator.setArmPower(angle);
    elevator.setPower(height);
    elevator.setExtend(extend);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - elevator.armAng()) < 4;
  }
}
