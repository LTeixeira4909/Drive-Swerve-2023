// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeShooterConstants;

public class CubeShooter extends SubsystemBase {
  private double m_angleSetpoint, m_topRollerSpeed, m_bottomRollerSpeed;
  private final CANSparkMax m_cubePivot, m_topRoller, m_bottomRoller;
  private static CubeShooter m_instance;

  private CubeShooterStates m_currentState, m_lastState;

  /** Creates a new CubeShooter. */
  public CubeShooter() {
    m_cubePivot = new CANSparkMax(CubeShooterConstants.PIVOT_MOTOR, MotorType.kBrushless);
    m_topRoller = new CANSparkMax(CubeShooterConstants.TOP_ROLLER_MOTOR, MotorType.kBrushless);
    m_bottomRoller = new CANSparkMax(CubeShooterConstants.BOTTOM_ROLLER_MOTOR, MotorType.kBrushless);

    m_currentState = CubeShooterStates.RETRACTED;
    m_cubePivot.setInverted(false);
  }

  public enum CubeShooterStates {
    CALIBRATE("Calibrate"), // only used to rezero the pivot angle
    RETRACTED("Retracted"), // inside fram perimiter, just off the hard stop
    INTAKE("Intake"), // picking cube up off the floor
    SPIT("Spit"), // eject a cube at the floor level for the hybrid node
    SCOREMID("ScoreMid"), // shooting angle
    SCOREHIGH("Scorehigh");

    String stateName;

    private CubeShooterStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return stateName;
    }
  }

  @Override
  public void periodic() {
    stateMachine();

    m_cubePivot.getPIDController().setReference(m_angleSetpoint, ControlType.kPosition);
    m_topRoller.set(m_topRollerSpeed);
    m_bottomRoller.set(m_bottomRollerSpeed);

  }

  private void stateMachine() {

    if (m_currentState.equals(m_lastState)) {
      // nothin to do
      return;
    }

    switch (m_currentState) {
      case CALIBRATE:

        break;
      case INTAKE:
        m_angleSetpoint = CubeShooterConstants.INTAKE_SETPOINT;
        m_bottomRollerSpeed = -.7;
        m_topRollerSpeed = -.7;
        break;
      case RETRACTED:
        m_angleSetpoint = CubeShooterConstants.RETRACTED_SETPOINT;
        m_bottomRollerSpeed = -.0;
        m_topRollerSpeed = -.0;
        break;
      case SCOREHIGH:
        m_angleSetpoint = CubeShooterConstants.SCOREHIGH_SETPOINT;
        m_bottomRollerSpeed = .75;
        m_topRollerSpeed = .75;
        break;
      case SCOREMID:
        m_angleSetpoint = CubeShooterConstants.SCOREMID_SETPOINT;
        m_bottomRollerSpeed = 0.45;
        m_topRollerSpeed = 0.45;
        break;
      case SPIT:
        m_angleSetpoint = CubeShooterConstants.SPIT_SETPOINT;
        m_bottomRollerSpeed = 0.2;
        m_topRollerSpeed = -0.2;
        break;
    }

  }

  public void setState(CubeShooterStates newState) {
    m_currentState = newState;
  }

  public static CubeShooter getInstance() {
    if (m_instance == null) {
      m_instance = new CubeShooter();
    }
    return m_instance;
  }
}
