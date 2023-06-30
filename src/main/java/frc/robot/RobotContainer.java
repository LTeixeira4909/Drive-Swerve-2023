// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.BalanceCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_operator = new CommandXboxController(1);

  private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_driver));
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // m_drivetrainSubsystem,
    // () -> -modifyAxis(m_controller.getLeftY()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(m_controller.getLeftX()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(m_controller.getRightX()) *
    // DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));
    // Configure the button bindings
    configureButtonBindings();

    // m_chooser.addOption("Complex Auto",
    // m_drivetrainSubsystem.traj(PathPlanner.loadPath("Angle Path", new
    // PathConstraints(4, 3)), true));

    addAutoPath("this path does not exist");
    addAutoPath("Rotate 90 Path");
    addAutoPath("1 Meter Path");
    addAutoPath("Back 1 Meter Path");
    addAutoPath("Back 180 Path");
    addAutoPath("simple charge station");
    addAutoPath("double score charge station");
    addAutoPath("double score charge station HP");
    m_chooser.setDefaultOption("Default Auto",
        m_drivetrainSubsystem.traj(PathPlanner.loadPath("1 Meter Path", new PathConstraints(4, 3)), true));
    SmartDashboard.putData(m_chooser);

  }

  public void addAutoPath(String pathName) {
    var fred = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));
    if (fred == null) {
      System.out.println("unable to load path" + pathName);
    } else {
      Command autoCmd = m_drivetrainSubsystem.traj(fred, true);
      autoCmd.setName(pathName);
      m_chooser.addOption(pathName, autoCmd);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driver.b().onTrue(new BalanceCommand());

    // Double square is back
    // 3 lines is start

    // we chose the lambda "() ->" format over the "::" format so we can pass values
    // to the methods which need them
    m_driver.back().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyro()));
    // This is another way to do the same as above:
    // m_driver.back().onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyro));

    m_driver.back().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyro()));

    m_driver.rightBumper().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.setDriveRate(0.3)))
        .onFalse(new InstantCommand(() -> m_drivetrainSubsystem.setDriveRate(1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Our Path", new
    // PathConstraints(4, 3)), true);
    // return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Left 7 Path", new
    // PathConstraints(4, 3)), true);
    // return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Angle Path", new
    // PathConstraints(4, 3)), true);
    // return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Rotate 90 Path", new
    // PathConstraints(4, 3)), true);

    return m_chooser.getSelected();
  }

  public String getSelectedAuto() {
    // String autoName = m_chooser.getSelected().getName();
    Command autoCmd = m_chooser.getSelected();
    if (autoCmd == null) {
      return "No Auto Selected";
    } else {
      return autoCmd.getName();
    }
  }

}
