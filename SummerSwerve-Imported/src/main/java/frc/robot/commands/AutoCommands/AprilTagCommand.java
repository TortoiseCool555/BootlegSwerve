// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ExtraMath;
import frc.robot.subsystems.NewSwerveDrivetrain;

public class AprilTagCommand extends CommandBase {
  /** Creates a new AprilTagCommand. */
  NewSwerveDrivetrain drive;
  public AprilTagCommand(NewSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tz = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tz").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("tid", id);
    double x = ExtraMath.clip(tx, 0.1);
    double y = ExtraMath.clip(ty, 0.1);
    double rot = ExtraMath.clip(ta, 0.1);
    SmartDashboard.putNumber("X Input", x);
    SmartDashboard.putNumber("Y Input", y);
    SmartDashboard.putNumber("Rot Input", rot);

    drive.setChassisSpeeds(x, y, rot);
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
