// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.cone1;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;

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

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...

  private final CommandXboxController m_controller = new CommandXboxController(0);
  // private final Joystick m_driver = new Joystick(0);

  private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
  
  private final ElevatorSubsytem m_elevatorSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_elevatorSubsystem = ElevatorSubsytem.getInstance();
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_controller.getHID()));
    //ElevatorSubsystem.setDefaultCommand(new ElevatorSubsystemCommand(m_controller.getHID()));
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
    m_controller.a().onTrue(new cone1(17720));
    m_controller.b().onTrue(new cone1(28813));
    m_controller.y().onTrue(new cone1(232));
  }
    
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Our Path", new PathConstraints(4, 3)), true);
    //return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Rotate 90 Path", new PathConstraints(4, 3)), true);
    //return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Charge Station Path", new PathConstraints(4, 3)), true);
    //return m_drivetrainSubsystem.traj(PathPlanner.loadPath("One piece charge Path", new PathConstraints(4, 3)), true);
    //return m_drivetrainSubsystem.traj(PathPlanner.loadPath("Two piece charge Path", new PathConstraints(2, 3)), true);
    //return m_drivetrainSubsystem.traj(PathPlanner.loadPath("three piece charge Path", new PathConstraints(5, 4)), true);
    //var examplePath = PathPlanner.loadPath((PathPlanner.loadPath("Charge Station Path", new PathConstraints(5, 4)), true);
    // var trajCmd = m_drivetrainSubsystem.traj(PathPlanner.loadPath("Charge Station Path", new PathConstraints(5, 4)), true);

    // FollowPathWithEvents command = new FollowPathWithEvents(trajCmd, null, null);
    
    // // An ExampleCommand will run in autonomous
    // // return m_autoCommand;
 
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("KingOfWorld", new PrintCommand("KingOfWorld"));
    
    // FollowPathWithEvents command2 = new FollowPathWithEvents(
    //     command,
    //     examplePath.getMarkers(),
    //     eventMap
    //   );
 
 
 
 
  }



  public void teleopPeriodic(){
    // m_intake.setDR();

  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0d)
        return (value - deadband) / (1d - deadband);
      else
        return (value + deadband) / (1d - deadband);
    } else {
      return 0d;
    }
  }

  private double modifyAxis(double value) {
    value = deadband(value, 0.05);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
 
    
 
 
 
 
 
 
 
 
 
  }





}
