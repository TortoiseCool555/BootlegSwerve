// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.InitDrivetrain;
import frc.robot.commands.AutoCommands.ResetForTele;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Park extends SequentialCommandGroup {
  /** Creates a new Park. */
  public Park(NewSwerveDrivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new DriveTime(drivetrain, -0.3, 0, 0, 3),
    new ResetForTele(drivetrain));
  }
}
