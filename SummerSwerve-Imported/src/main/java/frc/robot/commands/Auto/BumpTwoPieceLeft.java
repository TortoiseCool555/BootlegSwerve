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
import frc.robot.commands.AutoCommands.ResetForTele;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BumpTwoPieceLeft extends SequentialCommandGroup {
  /** Creates a new BumpTwoPieceLeft. */
  public BumpTwoPieceLeft(NewSwerveDrivetrain drivetrain, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double elHeight = 7000;
    double exPos = 17.1;
    // addCommands(new InitDrivetrainOdom(drivetrain, Math.toRadians(180)), new ResetElevator(elevator), 
    // new AutoControl(elevator, 0, 0, 95, true),
    // new AutoControl(elevator, elHeight, 0, 95, true),
    // new AutoControl(elevator, elHeight, exPos, 135, true),
    // new AutoControl(elevator, elHeight,exPos,135,true, 0), new Pause(elevator, elHeight, exPos, 135, 0.5),
    // new AutoControl(elevator, elHeight,exPos,135,false, 0.4), new Pause(elevator, elHeight, exPos, 135, 0.5),
    // new AutoControl(elevator, elHeight, 1, 90, false, 0),
    // new AutoControl(elevator, 0, 1, 90, false, 0),
    // new FollowPath(drivetrain, Paths.BumpScoreToCollectInBetweenLeft, false).raceWith(new AutoControlPower(elevator, 0, 0, 80, false)),
    // new FollowPath(drivetrain, Paths.BumpRotate, false).raceWith(new AutoControlPower(elevator, 0, 0, 80, false)),
    // new FollowPath(drivetrain, Paths.BumpRotate, false).raceWith(new AutoControlPower(elevator, 0, 0, 80, false)),
    // new DriveTime(drivetrain, 0.2, 0, 0, 1).raceWith(new AutoControlPower(elevator, 0, 0, 235, false, -0.35)),
    // new AutoControl(elevator, 0, 0, 75, false, 0),
    // new FollowPath(drivetrain, Paths.CollectToScoreInBetween, false).raceWith(new AutoControlPower(elevator, 0, 0, 75, false, 0)),
    // new FollowPath(drivetrain, Paths.CollectToScore, true).raceWith(new AutoControlPower(elevator, 0, 0, 75, false)),
    // new DriveTime(drivetrain, 0, 0, 0, 1).raceWith(new AutoControl(elevator, 0, 0, 75, false, 1))
    // );

    addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new ResetElevator(elevator), 
    new AutoControl(elevator, 0, 0, 80, true),
    new AutoControl(elevator, elHeight, 0, 95, true),
    new AutoControl(elevator, elHeight, exPos, 95, true),
    new AutoControl(elevator, elHeight,exPos,135,true, 0), new Pause(elevator, elHeight, exPos, 135, 1),
    new AutoControl(elevator, elHeight,exPos,135,false, 0.4), new Pause(elevator, elHeight, exPos, 135, 0.5),
    new AutoControl(elevator, elHeight, 1, 80, false, 0),
    new AutoControl(elevator, 0, 1, 80, false, 0),
    new DriveTime(drivetrain, -0.3 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 3.1).raceWith(new AutoControlPower(elevator, 0, 0, 73, false)),
    new DriveTime(drivetrain, 0, 0, 0.15 * Constants.MAX_ANG_RAD_PER_SEC, 2.2).raceWith(new AutoControlPower(elevator, 0, 0, 80, false)),
    new AutoControl(elevator, 0, 0, 225, false, 0.65),
    new DriveTime(drivetrain, -0.3 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 1.3).raceWith(new AutoControlPower(elevator, 0, 0, 230, false, 0.65)),
    new AutoControl(elevator, 0, 0, 75, false, 0.65),
    new ResetForTele(drivetrain)
    );
  }
}
