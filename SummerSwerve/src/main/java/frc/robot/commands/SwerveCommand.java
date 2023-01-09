// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class SwerveCommand extends CommandBase {
  /** Creates a new SwerveCommand. */
  NewSwerveDrivetrain drivetrain;
  XboxController controller;
  public SwerveCommand(XboxController controller, NewSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = controller.getLeftX();
    double y = -controller.getLeftY();
    double rot = controller.getRightX();

    if(Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(rot) < 0.1) {
      x = 0;
      y = 0;
      rot = 0;
    }

    drivetrain.setChassisSpeeds(x * Constants.MAX_TRANS_METERS_PER_SEC, 
    y * Constants.MAX_TRANS_METERS_PER_SEC, 
    rot * Constants.MAX_ANG_RAD_PER_SEC);

    SmartDashboard.putNumber("Yaw Angle", drivetrain.getAngle());
    SmartDashboard.putString("Module Angle Position Values", drivetrain.getModulePositionErrors());
    SmartDashboard.putString("Module Translation Positions", drivetrain.getModuleTranslationPositions());
    SmartDashboard.putString("Module Translation Velocities", drivetrain.getModuleVelocities());
    SmartDashboard.putString("Wanted Translation Velocities", drivetrain.getModuleWantedTranslationVelocity(x,y,rot));
    SmartDashboard.putString("Module Angular Power: ", drivetrain.getModulePositionPowers(x,y,rot));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
