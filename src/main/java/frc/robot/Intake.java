package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static Intake m_instance;

    private CANSparkMax m_DRFront;
    private CANSparkMax m_DRBack;

    private CANSparkMax m_WWRight;
    private CANSparkMax m_WWLeft;
    
    private Intake(){
        m_DRFront = new CANSparkMax(13, MotorType.kBrushless);
        m_DRBack = new CANSparkMax(14, MotorType.kBrushless);

        m_WWRight = new CANSparkMax(11, MotorType.kBrushless);
        m_WWLeft = new CANSparkMax(12, MotorType.kBrushless);
    }

    public static Intake getInstance(){
        return m_instance = (m_instance == null) ? new Intake() : m_instance;
    }

    public void setDR(double speed, boolean invert){
        m_DRFront.set(-speed);
        speed = invert ? speed : -speed;
        m_DRBack.set(speed);
    }


    public void setWW(double speed){
        m_WWRight.set(-speed);
        m_WWLeft.set(speed);
    }
}