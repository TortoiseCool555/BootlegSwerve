// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.element.Element;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CameraStream;
import frc.robot.commands.ElevatorDrive;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.Auto.Preload;
import frc.robot.commands.AutoCommands.AprilTagCommand;
import frc.robot.commands.AutoCommands.Balance;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.FollowPath;
import frc.robot.subsystems.CameraSubsystem;
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
  private final SwerveCommand swerveCommand = new SwerveCommand(controller, swerve);
  private final CameraSubsystem cam = new CameraSubsystem();
  private final CameraStream camStream = new CameraStream(cam);
  private final Elevator elevator = new Elevator(controller2);
  private final ElevatorDrive elCommand = new ElevatorDrive(elevator, controller2);
  // private final AprilTagCommand autoCommand = new AprilTagCommand(swerve);
  private final Preload autoCommand = new Preload(swerve, elevator);
  private final Balance balance = new Balance(swerve);
  private final FollowPath path = new FollowPath(swerve, Paths.SmC);
  private final DriveTime time = new DriveTime(swerve, -.3 * Constants.MAX_TRANS_METERS_PER_SEC, 0, 0, 300);

  //private final TestSubsystem test = new TestSubsystem(controller);
  //private final TestCommand tCom = new TestCommand(controller, test);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
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
    return autoCommand;
  }
}
