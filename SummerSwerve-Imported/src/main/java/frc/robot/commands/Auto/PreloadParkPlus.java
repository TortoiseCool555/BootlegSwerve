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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double elHeight = 7000;
    double exPos = 17.1;
    addCommands(new InitDrivetrain(drivetrain, 0, 0, Math.toRadians(180)), new ResetElevator(elevator), 
    new AutoControl(elevator, 0, 0, 95, true),
    new AutoControl(elevator, elHeight, 0, 95, true),
    new AutoControl(elevator, elHeight, exPos, 135, true),
    new AutoControl(elevator, elHeight,exPos,135,true, 0), new Pause(elevator, elHeight, exPos, 135, 0.5),
    new AutoControl(elevator, elHeight,exPos,135,false, 0.4), new Pause(elevator, elHeight, exPos, 135, 0.5),
    new AutoControl(elevator, elHeight, 1, 90, false, 0),
    new AutoControl(elevator, 0, 1, 90, false, 0),
    new FollowPath(drivetrain, Paths.ScoreToCollectInBetween, false).raceWith(new AutoControlPower(elevator, 0, 0, 90, false)),
    new FollowPath(drivetrain, Paths.ScoreToCollect, true).raceWith(new AutoControlPower(elevator, 0, 0, 180, false)),
    new DriveTime(drivetrain, 0.2, 0, 0, 1).raceWith(new AutoControlPower(elevator, 0, 0, 235, false, -0.35)),
    new AutoControl(elevator, 0, 0, 75, false, 0),
    new FollowPath(drivetrain, Paths.CollectToScoreInBetween, false).raceWith(new AutoControlPower(elevator, 0, 0, 75, false, 0)),
    new FollowPath(drivetrain, Paths.CollectToScore, true).raceWith(new AutoControlPower(elevator, 0, 0, 75, false)),
    new DriveTime(drivetrain, 0, 0, 0, 1).raceWith(new AutoControl(elevator, 0, 0, 75, false, 0.9))
    );
  }
}
