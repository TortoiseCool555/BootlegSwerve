// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;

public class CameraStream extends CommandBase {
  /** Creates a new CameraStream. */
  CameraSubsystem cam;
  double[] arr = {0.0,0.0,0.0,0.0,0.0,0.0};

  public CameraStream(CameraSubsystem cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cam);
    this.cam = cam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    double [] poseList = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(arr);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("tid", id);
    if(poseList.length >0){
      SmartDashboard.putNumber("x", poseList[0]);
      SmartDashboard.putNumber("y", poseList[1]);
      SmartDashboard.putNumber("z", poseList[2]);
      SmartDashboard.putNumber("rx", poseList[3]);
      SmartDashboard.putNumber("ry", poseList[4]);
      SmartDashboard.putNumber("rz", poseList[5]);

    }
    else{
      SmartDashboard.putNumber("x", 0);
      SmartDashboard.putNumber("y", 0);
      SmartDashboard.putNumber("z", 0);
      SmartDashboard.putNumber("rx", 0);
      SmartDashboard.putNumber("ry", 0);
      SmartDashboard.putNumber("rz", 0);
    }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
