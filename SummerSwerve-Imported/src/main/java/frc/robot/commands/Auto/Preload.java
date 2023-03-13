// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Paths;
import frc.robot.commands.AutoCommands.FollowPath;
import frc.robot.commands.AutoCommands.InitDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Preload extends SequentialCommandGroup {
  /** Creates a new Preload. */
  public Preload(NewSwerveDrivetrain drivetrain, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new PrepSolenoid(elevator, false), new AutoControl(elevator, 0, 0, 80, true),
    // new AutoControl(elevator, 11600, 0, 95, true),
    // new AutoControl(elevator, 11600, 17, 95, true),
    // new AutoControl(elevator,11600,17,120,false, 0.5),
    // new AutoControl(elevator, 11600, .5, 80, false, 0),
    // new AutoControl(elevator, 0, .5, 80, false, 0)
    // );
    // Balancing Portion
    // addCommands(new DriveTime(drivetrain, -0.23 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 2.1),
    // new Balance(drivetrain)
    addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new FollowPath(drivetrain, Paths.SmC)
    );
  }
}
