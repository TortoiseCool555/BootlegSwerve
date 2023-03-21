// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto.ChargeOnly;
import frc.robot.commands.Auto.ChargeStatiom;
import frc.robot.commands.Auto.Park;
import frc.robot.commands.Auto.Preload;
import frc.robot.commands.Auto.PureChargeNoPreload;
import frc.robot.commands.TeleOp.ZeroGyro;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);
  private final NewSwerveDrivetrain swerve = new NewSwerveDrivetrain(controller);
  private final Elevator elevator = new Elevator(controller2);

  // Autos
  private final Preload preloadAndPark = new Preload(swerve, elevator);
  private final ChargeStatiom chargeStation = new ChargeStatiom(swerve, elevator);
  private final ChargeOnly chargePreload = new ChargeOnly(swerve, elevator);
  private final PureChargeNoPreload chargeOnly = new PureChargeNoPreload(swerve, elevator);
  private final Park park = new Park(swerve);
  private final SendableChooser<Command> autoSelect = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    autoSelect.addOption("Preload and Charge Station", chargePreload);
    autoSelect.addOption("Charge Station", chargeOnly);
    autoSelect.addOption("Charge Station With Extra Points", chargeStation);
    autoSelect.addOption("Park", park);
    autoSelect.setDefaultOption("Preload and Park", preloadAndPark);
    SmartDashboard.putData(autoSelect);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(controller::getBackButton).onTrue(new ZeroGyro(swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return autoCommand;
    return autoSelect.getSelected();
  }
}
