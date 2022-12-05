package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Module {
    TalonFX driveMotorfl = new TalonFX(7);

    TalonFX turnMotorfl = new TalonFX(8);

    CANCoder encfl = new CANCoder(4);

    public Module() {
        turnMotorfl.configFactoryDefault();
         // set motor encoder to 0 when robot code starts
         turnMotorfl.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(encfl.getPosition()));
         turnMotorfl.setInverted(true);

         double turnMotorKp = .2;
         double turnMotorKI = 0;
         double turnMotorKD = 0.1;
 
         turnMotorfl.config_kP(0, turnMotorKp);
         turnMotorfl.config_kI(0, turnMotorKI);
         turnMotorfl.config_kD(0, turnMotorKD);
    
         driveMotorfl.setInverted(true);

         encfl.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encfl.setPosition(0);

    
        }
        public void setModuleState(SwerveModuleState state){
            driveMotorfl.set(ControlMode.PercentOutput, state.speedMetersPerSecond);
            
            turnMotorfl.set(ControlMode.Position, DrivetrainSubsystem.convertDegreesToTicks(state.angle.getDegrees()));       
            
        }


    }
