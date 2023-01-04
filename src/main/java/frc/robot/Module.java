package frc.robot;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Module {
    TalonFX driveMotor;
    TalonFX turnMotor;

    CANCoder enc;

    String name;
    // ShuffleboardTab tab;

    public Module(String name, int driveMotorCanId, int turnMotorCanId, int encoderCanId) {
        this.name = name;

        // this.tab = DrivetrainSubsystem.getInstance().getTab();

        driveMotor = new TalonFX(driveMotorCanId, "CANivore1");
        turnMotor = new TalonFX(turnMotorCanId, "CANivore1");
        enc = new CANCoder(encoderCanId, "CANivore1");

        enc.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        // enc.setStatusFramePeriod(statusFrame, periodMs)

        double initialOffset = 0;
        int maxRetryCount = 500;
        ErrorCode ec;
        do {
            initialOffset = enc.getPosition();
            ec = enc.getLastError();
            if (ec != ErrorCode.OK) {
                System.out.println(name + " -" + enc.getLastError());
            }
        } while (ec != ErrorCode.OK && maxRetryCount-- > 0);
        SmartDashboard.putNumber(name + "retrycount", maxRetryCount);

        turnMotor.configFactoryDefault();
        driveMotor.configFactoryDefault();
        // set motor encoder to 0 when robot code starts
        turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(initialOffset));
        turnMotor.setInverted(true);

        double turnMotorKp = .2;
        double turnMotorKI = 0;
        double turnMotorKD = 0.1;

        turnMotor.config_kP(0, turnMotorKp);
        turnMotor.config_kI(0, turnMotorKI);
        turnMotor.config_kD(0, turnMotorKD);

        driveMotor.setInverted(false);
        driveMotor.setNeutralMode(NeutralMode.Brake);

    }

    public final double MAX_VOLTAGE = 8;
    public final double NOMINAL_VOLTAGE = 12;

    public void setModuleState(SwerveModuleState desiredState) {

        // tab.add(name + " cancoder", enc.getPosition());
        // tab.add(name + " cancoder", enc.getPosition());
        SmartDashboard.putNumber(name + " Cancoder", enc.getPosition());
        SmartDashboard.putNumber(name + " turnMotor",
                DrivetrainSubsystem.convertTicksToDegrees(turnMotor.getSelectedSensorPosition()));

        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(enc.getPosition()));

        SmartDashboard.putNumber(name + " Heading", desiredState.angle.getDegrees());

        double desiredVoltage = (desiredState.speedMetersPerSecond / DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE ) / NOMINAL_VOLTAGE;
        SmartDashboard.putNumber(name + " desiredVoltage", desiredVoltage);

        driveMotor.set(ControlMode.PercentOutput, desiredVoltage);
        turnMotor.set(ControlMode.Position, DrivetrainSubsystem.convertDegreesToTicks(desiredState.angle.getDegrees()));

    }

    public SwerveModuleState getState() {

        // System.out.println("got here!");

        double speedTicksPer100miliSeconds = driveMotor.getSelectedSensorVelocity();

        double speedMetersPerSecond = speedTicksPer100miliSeconds * (1 / 100d) * (1_000 / 1d) * (1 / 2048d) * (1 / 6.75)
                * ((4.0 * Math.PI) / 1d) * (.0254 / 1);

        SmartDashboard.putNumber(name + "St", speedTicksPer100miliSeconds);
        SmartDashboard.putNumber(name + "Sm", speedMetersPerSecond);

        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(enc.getPosition()));

    }

    public void resetTurnEncoders() {
        enc.setPosition(0);
        turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(0));
    }
}
