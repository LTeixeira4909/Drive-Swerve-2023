// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private double m_angleSetpoint;
    private final CANSparkMax m_armPivot;
    private static Arm m_instance;

    private armStates m_currentState, m_lastState;

    public enum armStates {
        ARM_CALIBRATE("Calibrate"), // only used to rezero the pivot angle
        ARM_RETRACTED("Retracted"), // inside fram perimiter, just off the hard stop
        ARM_EXTENDED("Extended"), // picking cube up off the floor
        ARM_MID("Mid"), // eject a cube at the floor level for the hybrid node
        ARM_HIGH("High"),
        ARM_SPIT("Spit");

        String stateName;

        private armStates(String name) {
            this.stateName = name;
        }

        public String toString() {
            return stateName;
        }
    }

    /** Creates a new Arm. */
    public Arm() {
        m_armPivot = new CANSparkMax(ArmConstants.ARM_MOTOR, MotorType.kBrushless);

        m_currentState = armStates.ARM_RETRACTED;
        m_armPivot.setInverted(false);
    }

    @Override
    public void periodic() {
        stateMachine();
        m_armPivot.getPIDController().setReference(m_angleSetpoint, ControlType.kPosition);
    }

    private void stateMachine() {

        if (m_currentState.equals(m_lastState)) {
            return;
        }

        m_lastState = m_currentState;
        switch (m_currentState) {
            case ARM_CALIBRATE:
                m_angleSetpoint = ArmConstants.ARM_CALIBRATE;
                break;
            case ARM_RETRACTED:
                m_angleSetpoint = ArmConstants.ARM_RETRACTED;
                break;
            case ARM_EXTENDED:
                m_angleSetpoint = ArmConstants.ARM_EXTENDED;
                break;
            case ARM_MID:
                m_angleSetpoint = ArmConstants.ARM_MID;
                break;
            case ARM_HIGH:
                m_angleSetpoint = ArmConstants.ARM_HIGH;
                break;
            case ARM_SPIT:
                m_angleSetpoint = ArmConstants.ARM_SPIT;
                break;

        }
    }

    public void setArmState(armStates state) {
        m_currentState = state;
    }

    public static Arm getInstance() {
        if (m_instance == null) {
            m_instance = new Arm();
        }
        return m_instance;

    }
}
