// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;;

public class Intake {

    private final CANSparkMax m_intakeRollers;

    private IntakeStates m_currentState, m_lastState;

    public Intake(){
        m_intakeRollers = new CANSparkMax(IntakeConstants.INTAKE_ROLLLERS, MotorType.kBrushless);
    }
   
    public enum IntakeStates {
        INTAKE("Intake"),
        SPIT("Spit");

        private String nameOfState;

        private IntakeStates(String name) {
            this.nameOfState = name;
        }

        public String toString() {
            return nameOfState;
        }
    }

    // public class Intake {
        // private IntakeStates m_currentState, m_lastState;
        // private CANSparkMax m_intakeRollers;

    //     public Intake() {
    //         m_intakeRollers = new CANSparkMax(Intakeconstants.INTAKE_ROLLLERS, MotorType.kBrushless);
    //     }
    // }

    public void periodic() {
        stateMachine();
    }

    private void stateMachine() {
        if (m_currentState.equals(m_lastState)) {
            return;
        }

        switch (m_currentState){
            case INTAKE:
                m_intakeRollers.set(1);
                break;

            case SPIT:
                m_intakeRollers.set(-1);
                break;
        }
    }

    public void setState(IntakeStates newState) {
        m_currentState = newState;
    }
}