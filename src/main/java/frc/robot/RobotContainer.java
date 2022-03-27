// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.BottomCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.TopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  //both my controllers
  private final PS4Controller m_controller = new PS4Controller(Constants.ControllerPort);
  private final XboxController m_xbox = new XboxController(Constants.ControllerPort2);

  //climb system basically
  public static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public final ClimbCommand climbCommand = new ClimbCommand(m_xbox, m_elevatorSubsystem);

  public static ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final ArmCommands armCommands = new ArmCommands(m_xbox, m_armSubsystem);

  //getting the balls
  public static ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();
  public final BottomCommand bottomCommand = new BottomCommand(m_manipulatorSubsystem);
  public final TopCommand topCommand = new TopCommand(m_manipulatorSubsystem);

  public static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final IntakeCommand intakeCommand = new IntakeCommand(m_intakeSubsystem);
  public final StopIntakeCommand stopIntakeCommand = new StopIntakeCommand(m_intakeSubsystem);

  public final StopCommand stopCommand = new StopCommand(m_elevatorSubsystem, m_manipulatorSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, null
    ));
    climbCommand.schedule();
    armCommands.schedule();
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
    // Back button zeros the gyroscope
    new Button(m_controller::getTriangleButton)
     // No requirements because we don't need to interrupt anything
      .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    new Button(m_xbox::getXButton)
      .whenPressed(bottomCommand);
    new Button(m_xbox::getBButton)
      .whenPressed(topCommand);
    new Button(m_xbox::getAButton)
        .whenPressed(stopCommand);
    new Button(m_xbox::getYButton)
      .whenPressed(intakeCommand);
    new Button(m_xbox::getYButton)
      .whenReleased(stopIntakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
