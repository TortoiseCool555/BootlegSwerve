// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoControl;
import frc.robot.commands.AutoCommands.InitDrivetrain;
import frc.robot.commands.AutoCommands.Pause;
import frc.robot.commands.AutoCommands.ResetElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOnly extends SequentialCommandGroup {
  /** Creates a new ScoreOnly. */
  public ScoreOnly(NewSwerveDrivetrain drivetrain, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double elHeight = 7000;
    double exPos = 16;
    addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new ResetElevator(elevator), 
    new AutoControl(elevator, 0, 0, 80, true),
    new AutoControl(elevator, elHeight, 0, 95, true),
    new AutoControl(elevator, elHeight, exPos, 95, true),
    new AutoControl(elevator, elHeight,exPos,120,true, 0), new Pause(elevator, elHeight, exPos, 120, 1),
    new AutoControl(elevator, elHeight,exPos,120,false, 0.4), new Pause(elevator, elHeight, exPos, 120, 0.5),
    new AutoControl(elevator, elHeight, 0, 80, false, 0),
    new AutoControl(elevator, 0, 0, 75, false, 0)
    );
  }
}
