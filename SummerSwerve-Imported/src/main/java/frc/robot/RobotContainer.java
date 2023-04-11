// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto.PreloadCharge;
import frc.robot.commands.Auto.PreloadChargeOver;
import frc.robot.commands.Auto.PreloadMidCharge;
import frc.robot.commands.Auto.PreloadMidPark;
import frc.robot.commands.Auto.BumpTwoPieceLeft;
import frc.robot.commands.Auto.ChargeStatiom;
import frc.robot.commands.Auto.OdomTest;
import frc.robot.commands.Auto.Park;
import frc.robot.commands.Auto.PreloadPark;
import frc.robot.commands.Auto.PreloadParkPlus;
import frc.robot.commands.Auto.PreloadParkPlusLeft;
import frc.robot.commands.Auto.PureChargeNoPreload;
import frc.robot.commands.Auto.ScoreOnly;
import frc.robot.commands.Auto.StrafeOutAuto;
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
  private final PreloadPark preloadAndPark = new PreloadPark(swerve, elevator);
  private final ChargeStatiom chargeStation = new ChargeStatiom(swerve, elevator);
  private final PreloadCharge chargePreload = new PreloadCharge(swerve, elevator);
  private final PreloadChargeOver chargeOver = new PreloadChargeOver(swerve, elevator);
  private final PureChargeNoPreload chargeOnly = new PureChargeNoPreload(swerve, elevator);
  private final Park park = new Park(swerve);
  PreloadMidPark preloadMidPark = new PreloadMidPark(swerve, elevator);
  PreloadMidCharge preloadMidCharge = new PreloadMidCharge(swerve, elevator);
  private final SendableChooser<Command> autoSelect = new SendableChooser<>();
  private final ScoreOnly scoreOnly = new ScoreOnly(swerve, elevator);
  private final PreloadParkPlus preloadParkPlus = new PreloadParkPlus(swerve, elevator);
  private final PreloadParkPlusLeft preloadParkPlusLeft = new PreloadParkPlusLeft(swerve, elevator);
  OdomTest odomTest = new OdomTest(swerve);
  private final BumpTwoPieceLeft bumpTwoPieceLeft = new BumpTwoPieceLeft(swerve, elevator);
  private final StrafeOutAuto strafeOutAuto = new StrafeOutAuto(swerve, elevator);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    autoSelect.addOption("PreloadH and Charge Station", chargePreload);
    autoSelect.addOption("PreloadM and Charge Station", preloadMidCharge);
    autoSelect.addOption("Charge Staton Only", chargeOnly);
    autoSelect.addOption("PreloadH Charge and Park", chargeOver);
    // autoSelect.addOption("Charge Station With Extra Points", chargeStation);
    autoSelect.setDefaultOption("PreloadH and Park", preloadAndPark);
    autoSelect.setDefaultOption("PreloadM and Park", preloadMidPark);
    autoSelect.addOption("Park", park);
    autoSelect.addOption("Score Only", scoreOnly);
    // autoSelect.addOption("Odom Test", odomTest);
    autoSelect.addOption("2 Piece Right Side", preloadParkPlus);

    autoSelect.addOption("2 Piece Left Side", preloadParkPlusLeft);
    autoSelect.addOption("Strafe out of charge", strafeOutAuto);
    autoSelect.addOption("Bump Side", bumpTwoPieceLeft);
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
