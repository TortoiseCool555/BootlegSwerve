// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Paths;
import frc.robot.commands.AutoCommands.AutoControl;
import frc.robot.commands.AutoCommands.AutoControlPower;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.FollowPath;
import frc.robot.commands.AutoCommands.InitDrivetrain;
import frc.robot.commands.AutoCommands.InitDrivetrainOdom;
import frc.robot.commands.AutoCommands.Pause;
import frc.robot.commands.AutoCommands.ResetElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreloadParkPlus extends SequentialCommandGroup {
  /** Creates a new PreloadParkPlus. */
  public PreloadParkPlus(NewSwerveDrivetrain drivetrain, Elevator elevator) {
    double elHeight = 9500;
    double exPos = 17;
    addCommands(new InitDrivetrainOdom(drivetrain, Math.toRadians(180)), new ResetElevator(elevator), 
    // new AutoControl(elevator, 0, 0, 95, true),
    // new AutoControl(elevator, elHeight, 0, 95, true),
    new AutoControl(elevator, elHeight, exPos, 135, true),
    new AutoControl(elevator, elHeight,exPos,125,true, 0), new Pause(elevator, elHeight, exPos, 125, 0.1),
    new AutoControl(elevator, elHeight,exPos,125,false, 0.4), new Pause(elevator, elHeight, exPos, 125, 0.2),
    new AutoControl(elevator, elHeight, 0, 90.1, false, 0),
    // new AutoControl(elevator, 5000, 0, 90.1, false, 0),
    new DriveTime(drivetrain, 0, 0, 0, 0.2).raceWith(new AutoControl(elevator, 0, 0, 90.1, false, 0)),
    // new FollowPath(drivetrain, Paths.ScoreToCollectInBetween, false).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    new FollowPath(drivetrain, Paths.ScoreToCollect, true).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    // new AutoControl(elevator, 0, 0, 186, false, 0),
    new DriveTime(drivetrain, 0.2, 0, 0, 0.6).raceWith(new AutoControlPower(elevator, 0, 0, 225, false, -0.35)),
    new AutoControl(elevator, 0, 0,100, false, 0),
    // new FollowPath(drivetrain, Paths.CollectToScoreInBetween, false).raceWith(new AutoControlPower(elevator, 0, 0, 75, false, 0)),
    new FollowPath(drivetrain, Paths.CollectToScore, true).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    new DriveTime(drivetrain, 0, 0, 0, 0.5).raceWith(new AutoControlPower(elevator, 0, 0, 90, false, 0.4)),
    new FollowPath(drivetrain, Paths.ScoreToCollect2Inbetween, true).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    new FollowPath(drivetrain, Paths.ScoreToCollect2, true).raceWith(new AutoControlPower(elevator, 0, 0, 210, false)),
    new DriveTime(drivetrain, 0.2, 0, 0, 0.6).raceWith(new AutoControlPower(elevator, 0, 0, 225, false, -0.35)),
    new FollowPath(drivetrain, Paths.CollectToScore2Inbeween, true).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    new FollowPath(drivetrain, Paths.CollectToScore2, true).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    new DriveTime(drivetrain, 0, 0, 0, 0.3).raceWith(new AutoControlPower(elevator, 0, 0, 120, false, 0.4))
    );
  }
}
