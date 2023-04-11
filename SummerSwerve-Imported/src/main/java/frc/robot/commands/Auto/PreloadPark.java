// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.AutoControl;
import frc.robot.commands.AutoCommands.AutoControlPower;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.InitDrivetrain;
import frc.robot.commands.AutoCommands.Pause;
import frc.robot.commands.AutoCommands.ResetElevator;
import frc.robot.commands.AutoCommands.ResetForTele;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreloadPark extends SequentialCommandGroup {
  /** Creates a new Preload. */
  public PreloadPark(NewSwerveDrivetrain drivetrain, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // double elHeight = 8300;
    // addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new ResetElevator(elevator), 
    
    // new AutoControl(elevator, 0, 0, 80, true),
    // new AutoControl(elevator, elHeight, 0, 95, true),
    // new AutoControl(elevator, elHeight, 17, 95, true),
    // new AutoControl(elevator,elHeight,17,150,true, 0), new Pause(elevator, 8500, 17, 150, 1),
    // new AutoControl(elevator,elHeight,17,150,false, 0.4), new Pause(elevator, 8500, 17, 150, 0.5),
    // new AutoControl(elevator, elHeight, 1, 85, false, 0),
    // new AutoControl(elevator, 0, 1, 85, false, 0)
    // );

    // Over charge and back with score
    // double elHeight = 4000;
    // addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new ResetElevator(elevator), 
    // new AutoControl(elevator, 0, 0, 80, true),
    // new AutoControl(elevator, elHeight, 0, 95, true),
    // new AutoControl(elevator, elHeight, 15, 95, true),
    // new AutoControl(elevator, elHeight,15,140,true, 0), new Pause(elevator, elHeight, 17, 140, 1),
    // new AutoControl(elevator, elHeight,15,140,false, 0.4), new Pause(elevator, elHeight, 17, 140, 0.5),
    // new AutoControl(elevator, elHeight, 1, 85, false, 0),
    // new AutoControl(elevator, 0, 1, 75, false, 0),
    // new DriveTime(drivetrain, -0.3 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 3.2).raceWith(new AutoControlPower(elevator, 0, 0, 75, false))
    // );

    double elHeight = 7000;
    double exPos = 17.4;
    addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new ResetElevator(elevator), 
    new AutoControl(elevator, 0, 0, 80, true),
    new AutoControl(elevator, elHeight, 0, 95, true),
    new AutoControl(elevator, elHeight, exPos, 95, true),
    new AutoControl(elevator, elHeight,exPos,135,true, 0), new Pause(elevator, elHeight, exPos, 135, 1),
    new AutoControl(elevator, elHeight,exPos,135,false, 0.4), new Pause(elevator, elHeight, exPos, 135, 0.5),
    new AutoControl(elevator, elHeight, 1, 80, false, 0),
    new AutoControl(elevator, 0, 1, 80, false, 0),
    new DriveTime(drivetrain, -0.3 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 4).raceWith(new AutoControlPower(elevator, 0, 0, 73, false)),
    new ResetForTele(drivetrain)
    );

    // Balancing Portion
    // addCommands(new DriveTime(drivetrain, -0.23 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 2.1),
    // new Balance(drivetrain)

    // addCommands(new InitDrivetrain(drivetrain, 0, 0, 0), new FollowPath(drivetrain, Paths.SmC)
    // );
    // addCommands(new DriveTime(drivetrain, -0.3 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 2.1)
    // );
  }
}
