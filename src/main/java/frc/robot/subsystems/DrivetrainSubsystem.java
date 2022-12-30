package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Module;

public class DrivetrainSubsystem extends SubsystemBase {
    // private static DrivetrainSubsystem instance = null;

    double wheelBase = (31.625 * 0.0254) / 2.0;
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

    final double HEAD_LT_STEER_OFFSET = -Math.toRadians(0);
    final double HEAD_RT_STEER_OFFSET = -Math.toRadians(0);
    final double BACK_LT_STEER_OFFSET = -Math.toRadians(0);
    final double BACK_RT_STEER_OFFSET = -Math.toRadians(0);

    final int HEAD_LT_DRIVE_CAN_ID = 45;
    final int HEAD_RT_DRIVE_CAN_ID = 43;
    final int BACK_LT_DRIVE_CAN_ID = 23;
    final int BACK_RT_DRIVE_CAN_ID = 20;

    final int HEAD_LT_TURN_CAN_ID = 40;
    final int HEAD_RT_TURN_CAN_ID = 44;
    final int BACK_LT_TURN_CAN_ID = 41;
    final int BACK_RT_TURN_CAN_ID = 22;

    final int HEAD_LT_ENC_CAN_ID = 50;
    final int HEAD_RT_ENC_CAN_ID = 51;
    final int BACK_LT_ENC_CAN_ID = 52;
    final int BACK_RT_ENC_CAN_ID = 53;

    Module headLtModule = new Module("HeadLt", HEAD_LT_DRIVE_CAN_ID, HEAD_LT_TURN_CAN_ID, HEAD_LT_ENC_CAN_ID, HEAD_LT_STEER_OFFSET);
    Module headRtModule = new Module("HeadRt", HEAD_RT_DRIVE_CAN_ID, HEAD_RT_TURN_CAN_ID, HEAD_RT_ENC_CAN_ID, HEAD_RT_STEER_OFFSET);
    Module backRtModule = new Module("BackRt", BACK_LT_DRIVE_CAN_ID, BACK_LT_TURN_CAN_ID, BACK_LT_ENC_CAN_ID, BACK_LT_STEER_OFFSET);
    Module backLtModule = new Module("BackLt", BACK_RT_DRIVE_CAN_ID, BACK_RT_TURN_CAN_ID, BACK_RT_ENC_CAN_ID, BACK_RT_STEER_OFFSET);

    ShuffleboardTab drivetrainTab;

    SlewRateLimiter fwdBakRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter leftRightRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter turnRateLimiter = new SlewRateLimiter(0.5);

    Pigeon2 pigeon = new Pigeon2(0, "CANivore1");

    public Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public DrivetrainSubsystem() {
        pigeon.setYaw(0);

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroHeading(),
                new Pose2d(0, 0, new Rotation2d()));
    }

    // This is called every loop of the scheduler (~20ms)
    @Override
    public void periodic() {

        // Update the pose
        m_pose = m_odometry.update(getGyroHeading(), headLtModule.getState(), headRtModule.getState(),
                backLtModule.getState(), backRtModule.getState());

        SmartDashboard.putNumber("Pose roation", m_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Pose X", m_pose.getX());
        SmartDashboard.putNumber("Pose Y", m_pose.getY());

        DriveWithJoystick(js0);
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

        // set max speed
        turn *= .4;
        fwdBackDir *= .6;
        leftRightDir *= .6;

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

        // button 8 on xbox is three lines button
        if (js.getRawButton(8)) {
            headLtModule.resetTurnEncoders();
            headRtModule.resetTurnEncoders();
            backRtModule.resetTurnEncoders();
            backLtModule.resetTurnEncoders();
        }
        // button 7 on xbox it two squares
        if (js.getRawButton(7)) {
            pigeon.setYaw(0);
        }

        // Example chassis speeds: 1 meter per second forward, 3 meters
        // per second to the left, and rotation at 1.5 radians per second
        // counteclockwise.
        // ChassisSpeeds speeds = new ChassisSpeeds(fwdBackDir, leftRightDir, turn);

        SmartDashboard.putNumber("Pidgeon yaw", pigeon.getYaw());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwdBackDir, leftRightDir, turn,
                Rotation2d.fromDegrees(pigeon.getYaw()));

        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backRight = moduleStates[3];
        SwerveModuleState backLeft = moduleStates[2];

        headLtModule.setModuleState(frontLeft);
        headRtModule.setModuleState(frontRight);
        backRtModule.setModuleState(backRight);
        backLtModule.setModuleState(backLeft);

    }

}
