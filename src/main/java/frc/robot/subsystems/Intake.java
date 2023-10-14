// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_intakeRollers;
    private IntakeStates m_currentState, m_lastState;
    private double m_rollerSpeed;
    private double m_currentLimit;

    public Intake() {
        m_intakeRollers = new CANSparkMax(IntakeConstants.INTAKE_ROLLLERS, MotorType.kBrushless);
        m_currentState = IntakeStates.IDLE;
        m_intakeRollers.setSmartCurrentLimit(60);
    }

    public enum IntakeStates {
        INTAKE("Intake"),
        SPIT("Spit"),
        IDLE("Idle"),
        HYBRID("Hybrid"), 
        SLOWINTAKE("");


        private String nameOfState;

        private IntakeStates(String name) {
            this.nameOfState = name;
        }

        public String toString() {
            return nameOfState;
        }
    }

    public void periodic() {
        stateMachine();
        SmartDashboard.putNumber("intake/rollerSpeed", m_rollerSpeed);
        SmartDashboard.putString("intake/CurrentState", m_currentState.toString());
        SmartDashboard.putNumber("intake/current", m_intakeRollers.getOutputCurrent());
    }

    private void stateMachine() {
        if (m_currentState.equals(m_lastState)) {
            return;
        }

        switch (m_currentState) {
            case INTAKE:
                m_rollerSpeed = IntakeConstants.INTAKE;
                break;

            case SPIT:
                m_rollerSpeed = IntakeConstants.SPIT;
                break;

            case IDLE:
                m_rollerSpeed = IntakeConstants.IDLE;
                break;

            case HYBRID:
                m_rollerSpeed = IntakeConstants.HYBRID;
                break;
            case SLOWINTAKE:
                m_rollerSpeed = -0.1;
                break;
        }

        m_intakeRollers.set(m_rollerSpeed);
    }

    public void setIntakeState(IntakeStates newState) {
        m_currentState = newState;
    }

}
