// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ExtraMath;
import frc.robot.subsystems.Elevator;

public class AutoControl extends CommandBase {
  /** Creates a new AutoControl. */
  Elevator elevator;
  Timer timer = new Timer();
  double angle;
  double elPos;
  double exPos;
  double inPow = 0;
  boolean state;
  boolean check1;
  boolean check2;
  boolean check3;
  boolean check4;
  double time;
  boolean goingUp = true;
  boolean isClose = false;
  public AutoControl(Elevator elevator, double elPos, double exPos, double angle, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
    this.elPos = elPos;
    this.exPos = exPos;
    this.angle = angle;
    this.state = state;
  }
  public AutoControl(Elevator elevator, double elPos, double exPos, double angle, boolean state, double inPow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
    this.elPos = elPos;
    this.exPos = exPos;
    this.angle = angle;
    this.state = state;
    this.inPow = inPow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goingUp = elPos - (-elevator.getPosition()) > 0 ? true : false;
    isClose = Math.abs(elPos - (-elevator.getPosition())) < 300;
    check4 = false;
    elevator.setState(state);
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    check1 = Math.abs(elPos + elevator.getPosition()) < 200;
    // check1 = true;
    check2 = Math.abs(exPos - elevator.getExtDist()) < 1.7;
    check3 = Math.abs(angle - elevator.getArmAngle()) < 5;
    check4 = check1 && check2 && check3;

    SmartDashboard.putNumber("Arm Angle", elevator.getArmAngle());
    SmartDashboard.putNumber("Desired Angle", angle);
    SmartDashboard.putNumber("Angle Error", elevator.getArmAngle() - angle);
    SmartDashboard.putNumber("Extension", elevator.getExtDist());
    SmartDashboard.putNumber("Desired Extension", exPos);
    SmartDashboard.putNumber("Elevator Position AUTO:", elevator.getPosition());
    SmartDashboard.putNumber("Elevator Power AUTO:", ExtraMath.clip((-elPos - elevator.getPosition())/4500.0, 0.5));
    SmartDashboard.putNumber("Target Position AUTO: ", elPos);

    if(goingUp && !isClose) {
      elevator.setExtendConstrainedScore(exPos, angle);
    } else {
      elevator.setExtend(exPos);
    }
    elevator.setArmAngle(angle);
    elevator.setPosition(elPos, true);
    elevator.setIntake(inPow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check4;
  }
}
