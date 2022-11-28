package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

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
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    Joystick js0 = new Joystick(0);

    TalonFX driveMotor = new TalonFX(5);
 
    TalonFX turnMotor = new TalonFX(6);
    
    CANCoder enc = new CANCoder(3);
    
    public DrivetrainSubsystem() {
        turnMotor.configFactoryDefault();

        // set motor encoder to 0 when robot code starts
        turnMotor.setSelectedSensorPosition(0);
    
        turnMotor.config_kP(0, 0.2);
        turnMotor.config_kI(0, 0);
        turnMotor.config_kD(0, 0.1);

        // turnMotor.selec

        enc.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    
    }
        @Override
        public void periodic() {
       DriveWithJoystick(js0);
       
   }

    public double convertTicksToDegrees(double ticks) {
        double degrees = ticks * (1.0/2048.0) * (1.0/12.8) * (360.0/1.0);
        return degrees;
    }    

    public double convertDegreesToTicks(double degrees) {

        double ticks = degrees * 1 / ( (1.0/2048.0) * (1.0/12.8) * (360.0/1.0) );
        return ticks;
    }

    public void DriveWithJoystick(Joystick js) {
      
 
        double x = js. getRawAxis(0); 

        double y = js. getRawAxis(1);

        double z = js. getRawAxis(2);
        
       
       
       // Example chassis speeds: 1 meter per second forward, 3 meters
        // per second to the left, and rotation at 1.5 radians per second
        // counteclockwise.
        ChassisSpeeds speeds = new ChassisSpeeds(x,y,z);

        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0];

        // Front right module state
        SwerveModuleState frontRight = moduleStates[1];

        // Back left module state
        SwerveModuleState backLeft = moduleStates[2];

        // Back right module state
        SwerveModuleState backRight = moduleStates[3];

        SmartDashboard.putNumber("front left speed", frontLeft.speedMetersPerSecond);
        
        SmartDashboard.putNumber("front left heading", frontLeft.angle.getDegrees());
        
        driveMotor.set(ControlMode.PercentOutput, frontLeft.speedMetersPerSecond);
        frontLeft.angle.getDegrees();

        turnMotor.set(
            ControlMode.Position,
            convertDegreesToTicks(
                frontLeft.angle.getDegrees()
            )
        );
        SmartDashboard.putNumber("PID Error", turnMotor.getClosedLoopError());
        
        // turnMotor.set(ControlMode.PercentOutput, .5);

        // t2.set(ControlMode.Position,frontLeft.speedMetersPerSecond);

       // t2.getSelectedSensorPosition()

       double ticks = turnMotor.getSelectedSensorPosition();

       double degrees = ticks * (1.0/2048.0) * (1.0/12.8) * (360.0/1.0);
       
        SmartDashboard.putNumber("t2", degrees % 360);
        
        SmartDashboard.putNumber("t2 ticks", ticks);
        
        SmartDashboard.putNumber("t2 Degrees", degrees);
    
        SmartDashboard.putNumber("enc", enc.getPosition()); 
   
        SmartDashboard.putString("enc this is the error", enc.getLastError().name());
   
        

        
   
   
    } 


}
        

