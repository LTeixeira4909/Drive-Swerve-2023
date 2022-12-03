package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private static DrivetrainSubsystem instance = null;

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    Joystick js0 = new Joystick(0);

    TalonFX driveMotorfl = new TalonFX(7);

    TalonFX turnMotorfl = new TalonFX(8);

    TalonFX turnMotorfr = new TalonFX(1);

    TalonFX driveMotorfr = new TalonFX(2);

    CANCoder enc = new CANCoder(4);

    public DrivetrainSubsystem() {
        turnMotorfl.configFactoryDefault();

        turnMotorfr.configFactoryDefault();
        // set motor encoder to 0 when robot code starts
        turnMotorfl.setSelectedSensorPosition(0);
        turnMotorfl.setInverted(true);

        turnMotorfr.setSelectedSensorPosition(0);
        turnMotorfr.setInverted(true);

        // driveMotor.setInverted(true);

        turnMotorfl.config_kP(0, 0.2);
        turnMotorfl.config_kI(0, 0);
        turnMotorfl.config_kD(0, 0.1);

        turnMotorfr.config_kP(0, 0.2);
        turnMotorfr.config_kI(0, 0);
        turnMotorfr.config_kD(0, 0.1);

        driveMotorfr.setInverted(true);

        enc.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        enc.setPosition(0);

    }

    @Override
    public void periodic() {
        DriveWithJoystick(js0);
    }

    public double convertTicksToDegrees(double ticks) {
        double degrees = ticks * (1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0);
        return degrees;
    }

    public double convertDegreesToTicks(double degrees) {

        double ticks = degrees * 1 / ((1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0));
        return ticks;
    }

    public void DriveWithJoystick(Joystick js) {

        double leftRightDir = -1 * js.getRawAxis(0);

        double fwdBackDir = -1 * js.getRawAxis(1);

        double z = js.getRawAxis(2);

        if (-0.1 < leftRightDir && leftRightDir < 0.1) {
            leftRightDir = 0;
        }

        if (-0.1 < fwdBackDir && fwdBackDir < 0.1) {
            fwdBackDir = 0;
        }

        if (-0.1 < z && z < 0.1) {
            z = 0;
        }

        SmartDashboard.putNumber("leftRightDir number", leftRightDir);

        SmartDashboard.putNumber("fwdBackDir number", fwdBackDir);

        SmartDashboard.putNumber("Z number", z);
        // Example chassis speeds: 1 meter per second forward, 3 meters
        // per second to the left, and rotation at 1.5 radians per second
        // counteclockwise.
        ChassisSpeeds speeds = new ChassisSpeeds(fwdBackDir, leftRightDir, z);

        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0]; // SwerveModuleState.optimize(moduleStates[0], new
                                                       // Rotation2d(enc.getPosition()));;

        // Front right module state
        SwerveModuleState frontRight = moduleStates[1];

        // Back left module state
        // SwerveModuleState backLeft = moduleStates[2];

        // Back right module state
        // SwerveModuleState backRight = moduleStates[3];

        SmartDashboard.putNumber("front left speed", frontLeft.speedMetersPerSecond);

        SmartDashboard.putNumber("front left heading", frontLeft.angle.getDegrees());

        driveMotorfl.set(ControlMode.PercentOutput, frontLeft.speedMetersPerSecond);

        driveMotorfr.set(ControlMode.PercentOutput, frontLeft.speedMetersPerSecond);
        // frontLeft.angle.getDegrees();

        double desiredDegrees = frontLeft.angle.getDegrees();
        double desiredTicks = convertDegreesToTicks(desiredDegrees);
        SmartDashboard.putNumber("Desired degrees", desiredDegrees);
        SmartDashboard.putNumber("Desired ticks", desiredTicks);

        turnMotorfl.set(ControlMode.Position, desiredTicks);

        turnMotorfr.set(ControlMode.Position, desiredTicks);
        // SmartDashboard.putNumber("PID Error", turnMotor.getClosedLoopError());

        // turnMotor.set(ControlMode.PercentOutput, .5);

        // t2.set(ControlMode.Position,frontLeft.speedMetersPerSecond);

        // t2.getSelectedSensorPosition()

        // SmartDashboard.putNumber("actual tick function",
        // convertDegreesToTicks(frontLeft.angle.getDegrees()));

        double actualTicks = turnMotorfl.getSelectedSensorPosition();
        double actualDegrees = convertTicksToDegrees(actualTicks);

        // double actualTicks = turnMotorfr.getSelectedSensorPosition();
        // double actualDegrees = convertTicksToDegrees(actualTicks);

        SmartDashboard.putNumber("Actual degrees", actualDegrees);
        SmartDashboard.putNumber("Actual ticks", actualTicks);

        SmartDashboard.putNumber("encoder degrees", enc.getPosition());

        // SmartDashboard.putString("enc this is the error", enc.getLastError().name());

    }

}
