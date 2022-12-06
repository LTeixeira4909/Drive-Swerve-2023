package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Module {
    TalonFX driveMotor;

    TalonFX turnMotor;

    CANCoder enc;

    int encoderCanId;

    public Module(int driveMotorCanId, int turnMotorCanId, int encoderCanId) {
        this.encoderCanId = encoderCanId;

        driveMotor = new TalonFX(driveMotorCanId);

        turnMotor = new TalonFX(turnMotorCanId);

        enc = new CANCoder(encoderCanId);

        turnMotor.configFactoryDefault();
        driveMotor.configFactoryDefault();
        // set motor encoder to 0 when robot code starts
        turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(enc.getPosition()));
        turnMotor.setInverted(true);

        double turnMotorKp = .2;
        double turnMotorKI = 0;
        double turnMotorKD = 0.1;

        turnMotor.config_kP(0, turnMotorKp);
        turnMotor.config_kI(0, turnMotorKI);
        turnMotor.config_kD(0, turnMotorKD);

        driveMotor.setInverted(true);

        enc.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // enc.setPosition(0);

    }

    public void setModuleState(SwerveModuleState desiredState) {

        SmartDashboard.putNumber("canCoder "+encoderCanId, enc.getPosition());

        // desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(enc.getPosition()));

        driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond);

        turnMotor.set(ControlMode.Position, DrivetrainSubsystem.convertDegreesToTicks(desiredState.angle.getDegrees()));

    }

}
