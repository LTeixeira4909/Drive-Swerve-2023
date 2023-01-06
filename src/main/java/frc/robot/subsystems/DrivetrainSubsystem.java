package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Module;

public class DrivetrainSubsystem extends SubsystemBase {
    // private static DrivetrainSubsystem instance = null;

    double wheelBase = 31.625 * 0.0254;
    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(wheelBase, wheelBase);
    Translation2d m_frontRightLocation = new Translation2d(wheelBase, -wheelBase);
    Translation2d m_backLeftLocation = new Translation2d(-wheelBase, wheelBase);
    Translation2d m_backRightLocation = new Translation2d(-wheelBase, -wheelBase);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation);

    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry;

    Pose2d m_pose;

    Joystick js0 = new Joystick(0);

    Module leftModule = new Module("FrontLeft", 7, 8, 14);
    Module rightModule = new Module("FrontRight", 2, 1, 11);
    Module backRightModule = new Module("BackRight", 4, 3, 12);
    Module backLeftModule = new Module("BackLeft", 6, 5, 13);

    ShuffleboardTab drivetrainTab;

    SlewRateLimiter fwdBakRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter leftRightRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter turnRateLimiter = new SlewRateLimiter(0.5);

    Pigeon2 pigeon = new Pigeon2(20, "CANivore1");

    public Rotation2d getGyroHeading() {
        // // Get my gyro angle. We are negating the value because gyros return positive
        // // values as the robot turns clockwise. This is not standard convention that
        // is
        // // used by the WPILib classes.
        // var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    private Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    private static DrivetrainSubsystem m_inst = null;

    public static DrivetrainSubsystem getInstance() {
        if (m_inst == null) {
            m_inst = new DrivetrainSubsystem();
        }
        return m_inst;
    }

    private DrivetrainSubsystem() {
        pigeon.setYaw(0);

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroHeading(),
                new Pose2d(0, 0, new Rotation2d()));
        
        SmartDashboard.putBoolean("Done", false);
    }

    // private DrivetrainSubsystem() {
    // drivetrainTab = Shuffleboard.getTab("drivetrain");

    // }

    // public static DrivetrainSubsystem getInstance() {
    // if (instance == null) {
    // instance = new DrivetrainSubsystem();
    // }
    // return instance;
    // }

    // public ShuffleboardTab getTab() {
    // return drivetrainTab;
    // }

    // this is alled every loop of the scheduler (~20ms)
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Pidgeon yaw", pigeon.getYaw());

        // Update the pose
        // button 8 on xbox is three lines button
        if (js0.getRawButton(8)) {
            leftModule.resetTurnEncoders();
            rightModule.resetTurnEncoders();
            backRightModule.resetTurnEncoders();
            backLeftModule.resetTurnEncoders();
        }
        // button 7 on xbox it two squares
        if (js0.getRawButton(7)) {
            pigeon.setYaw(0);
        }

        SmartDashboard.putString("odo", m_odometry.getPoseMeters().toString());
        SmartDashboard.putString("odo", m_odometry.getPoseMeters().toString());
        // SmartDashboard.putNumber("Pose roation", m_pose.getRotation().getDegrees()) ;
        // SmartDashboard.putNumber("Pose X", m_pose.getX());
        // SmartDashboard.putNumber("Pose Y", m_pose.getY());

        // DriveWithJoystick(js0);
        m_odometry.update(getGyroHeading(), leftModule.getState(), rightModule.getState(),
                backLeftModule.getState(), backRightModule.getState());
    }

    public static double convertTicksToDegrees(double ticks) {
        double degrees = ticks * (1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0);
        return degrees;
    }

    public static double convertDegreesToTicks(double degrees) {

        double ticks = degrees * 1 / ((1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0));
        return ticks;
    }

    public void DriveWithJoystick(Joystick js) {

        double leftRightDir = -1 * js.getRawAxis(0); // positive number means left
        double fwdBackDir = -1 * js.getRawAxis(1); // positive number means fwd
        double turn = -1 * js.getRawAxis(4); // positive number means clockwise

        // fwdBackDir = fwdBakRateLimiter.calculate(fwdBackDir);
        // leftRightDir = leftRightRateLimiter.calculate(leftRightDir);
        // turn = turnRateLimiter.calculate(turn);

        double deadband = .05;

        if (-deadband < leftRightDir && leftRightDir < deadband) {
            leftRightDir = 0;
        }

        if (-deadband < fwdBackDir && fwdBackDir < deadband) {
            fwdBackDir = 0;
        }

        if (-deadband < turn && turn < deadband) {
            turn = 0;
        }

        // Example chassis speeds: 1 meter per second forward, 3 meters
        // per second to the left, and rotation at 1.5 radians per second
        // counteclockwise.
        // ChassisSpeeds speeds = new ChassisSpeeds(fwdBackDir, leftRightDir, turn);

        

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fwdBackDir * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                leftRightDir * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                turn * DrivetrainSubsystem.MAX_OMEGA_RADIANS_PER_SECOND,
                Rotation2d.fromDegrees(pigeon.getYaw()));

        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        setModuleStates(moduleStates);

    }

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4;
    public static final double MAX_OMEGA_RADIANS_PER_SECOND = 1;

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backRight = moduleStates[3];
        SwerveModuleState backLeft = moduleStates[2];

        leftModule.setModuleState(frontLeft);
        rightModule.setModuleState(frontRight);
        backRightModule.setModuleState(backRight);
        backLeftModule.setModuleState(backLeft);
    }

    public Command traj(PathPlannerTrajectory traj, boolean isFirstPath) {

        PIDController xyController = new PIDController(5, 0, 0);

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath)
                        this.m_odometry.resetPosition(traj.getInitialHolonomicPose(), getGyroHeading());
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose,
                        this.m_kinematics,
                        xyController,
                        xyController,
                        new PIDController(5, 0, 0),
                        this::setModuleStates,
                        this),
                new InstantCommand(() -> {
                    SmartDashboard.putBoolean("Done", true);
                }));
    }

}
