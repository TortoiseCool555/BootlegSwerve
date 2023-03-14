// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class Pause extends CommandBase {
  /** Creates a new Pause. */
  Timer timer = new Timer();
  double pos, extend, angle, pause;
  Elevator elevator;
  public Pause(Elevator elevator, double pos, double extend, double angle, double pause) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pause = pause;
    this.elevator = elevator;
    this.pos = pos;
    this.extend = extend;
    this.angle = angle;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPosition(pos, true);
    elevator.setArmAngle(angle);
    elevator.setExtend(extend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > pause;
  }
}
