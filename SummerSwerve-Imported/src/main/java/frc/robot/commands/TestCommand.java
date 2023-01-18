// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class TestCommand extends CommandBase {
  /** Creates a new TestCommand. */
  XboxController controller;
  TestSubsystem test;
  public TestCommand(XboxController controller, TestSubsystem test) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(test);
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    test.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getAButton()){
      test.setMotor(1, true);
    }
    else if(controller.getBButton()){
      test.setMotor(1, false);
    }
    else{
      test.setMotor(0, true);
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
