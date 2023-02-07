// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.VideoEvent.Kind;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsytem extends SubsystemBase {
  public CANSparkMax m_leadMotor;
  public CANSparkMax m_followMotor;

  public static final int leadDeviceID = 1;
  public static final int followDeviceID = 2;
  public CANSparkMax m_motor;
  public SparkMaxPIDController m_pidController;
  public RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private static ElevatorSubsytem m_inst = null;

  public static ElevatorSubsytem getInstance() {
    if (m_inst == null) {
      m_inst = new ElevatorSubsytem();
    }
    return m_inst;
  }

  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsytem() {
    m_leadMotor = new CANSparkMax(leadDeviceID, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(followDeviceID, MotorType.kBrushless);

    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_followMotor.follow(m_leadMotor);

    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setSetpoint(double distance) {
    // m_pid
    m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);
  }

}
