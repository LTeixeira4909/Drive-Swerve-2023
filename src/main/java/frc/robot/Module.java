package frc.robot;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
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

    public Module(String name, int driveMotorCanId, int turnMotorCanId, int encoderCanId, double encoderOffsetDegrees) {
        this.name = name;

        enc = new CANCoder(encoderCanId, "CANivore1");
        driveMotor = new TalonFX(driveMotorCanId, "CANivore1");
        turnMotor = new TalonFX(turnMotorCanId, "CANivore1");

        final int CAN_TIMEOUT_MS = 250;

        // Encoder Configuration
        {
            var config = new CANCoderConfiguration();
            config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            // config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360; // from
            // SDS sample code
            // config.sensorDirection = direction == Direction.CLOCKWISE; // from SDS sample
            // code
            config.magnetOffsetDegrees = encoderOffsetDegrees;

            checkErrorCode(enc.configAllSettings(config, CAN_TIMEOUT_MS), "Unable to configure " + name + " Cancoder!");
        }

        // BB: I think the default is 10ms which is a good value if we are going to use
        // the encoder as the loop time is 20ms.
        // var periodMilliseconds = 10;
        // enc.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, CAN_TIMEOUT_MS);

        // Drive Motor Configuration
        {
            checkErrorCode(driveMotor.configFactoryDefault(), "Unable to clear config of " + name + " drive motor!");

            var config = new TalonFXConfiguration();
            config.voltageCompSaturation = 12;
            config.supplyCurrLimit.currentLimit = 80;
            config.supplyCurrLimit.enable = true;
            checkErrorCode(driveMotor.configAllSettings(config, CAN_TIMEOUT_MS),
                    "Unable to configure " + name + " drive motor!");

            driveMotor.enableVoltageCompensation(true);
            checkErrorCode(enc.getLastError(), "Unable to enable voltage compenstation on " + name + " turn motor!");

            driveMotor.setNeutralMode(NeutralMode.Brake);
            checkErrorCode(enc.getLastError(), "Unable to set brake mode on " + name + " turn motor!");

            // reduce status frame messages
            final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
            checkErrorCode(driveMotor.setStatusFramePeriod(
                    StatusFrameEnhanced.Status_1_General,
                    STATUS_FRAME_GENERAL_PERIOD_MS,
                    CAN_TIMEOUT_MS), "Unable to change status frame for " + name + " drive motor!");
        }

        // Turn Motor Configuration
        {
            checkErrorCode(turnMotor.configFactoryDefault(), "Unable to clear config of " + name + " turn motor!");

            var config = new TalonFXConfiguration();
            config.slot0.kP = 0.2;
            config.slot0.kI = 0.0;
            config.slot0.kD = 0.1;
            config.voltageCompSaturation = 12.0;
            config.supplyCurrLimit.currentLimit = 20.0;
            config.supplyCurrLimit.enable = true;
            checkErrorCode(turnMotor.configAllSettings(config, CAN_TIMEOUT_MS),
                    "Unable to config of " + name + " turn motor!");

            driveMotor.enableVoltageCompensation(true);
            checkErrorCode(enc.getLastError(), "Unable to enable voltage compenstation on " + name + " turn motor!");

            turnMotor.setInverted(true);
            checkErrorCode(enc.getLastError(), "Unable to set " + name + " turn motor inverted!");

            final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
            checkErrorCode(driveMotor.setStatusFramePeriod(
                    StatusFrameEnhanced.Status_1_General,
                    STATUS_FRAME_GENERAL_PERIOD_MS,
                    CAN_TIMEOUT_MS), "Unable to change status frame for " + name + " drive motor!");
        }

        double currentHeading = enc.getAbsolutePosition();
        checkErrorCode(enc.getLastError(), "Unable to get " + name + " turn motor current heading!");

        // set motor encoder to wheel heading as measured by encoder
        turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(currentHeading));

    }

    public void setModuleState(SwerveModuleState desiredState) {

        // tab.add(name + " cancoder", enc.getPosition());
        // tab.add(name + " cancoder", enc.getPosition());
        SmartDashboard.putNumber(name + " Cancoder", enc.getPosition());
        SmartDashboard.putNumber(name + " turnMotor",
                DrivetrainSubsystem.convertTicksToDegrees(turnMotor.getSelectedSensorPosition()));

        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(enc.getPosition()));

        SmartDashboard.putNumber(name + " Heading", desiredState.angle.getDegrees());

        driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond);
        turnMotor.set(ControlMode.Position, DrivetrainSubsystem.convertDegreesToTicks(desiredState.angle.getDegrees()));

    }

    public SwerveModuleState getState() {
        double speedTicksPer100miliseconds = driveMotor.getSelectedSensorVelocity();

        double speedMetersPerSecond = speedTicksPer100miliseconds * (10.0 / 1.0) * (1.0 / 2048.0) * (1.0 / 6.75)
                * (Math.PI / 1.0) * (.0254 / 1.0);

        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(enc.getPosition()));
    }

    public void resetTurnEncoders() {
        enc.setPosition(0);
        turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(0));
    }

    // Should only be used when initalizing, or you may throw errors when driving.
    private void checkErrorCode(ErrorCode ec, String errorMsg) {
        if (ec != ErrorCode.OK) {
            // @todo don't do this when connected to FMS.
            throw new Error(errorMsg + "- Reason: " + ec.toString());
        }
    }
}
