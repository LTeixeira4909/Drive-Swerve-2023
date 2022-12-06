package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Module {
    TalonFX driveMotor;

    TalonFX turnMotor;

    CANCoder enc;

    public Module(int driveMotorCanId, int turnMotorCanId, int encoderCanId) {
        driveMotor = new TalonFX(driveMotorCanId);
        
        turnMotor = new TalonFX(turnMotorCanId);

        enc = new CANCoder(encoderCanId);
        
        turnMotor.configFactoryDefault();
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
        enc.setPosition(0);

    
        }
        public void setModuleState(SwerveModuleState state){
            driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond);
            
            turnMotor.set(ControlMode.Position, DrivetrainSubsystem.convertDegreesToTicks(state.angle.getDegrees()));       
            
        }


    }
