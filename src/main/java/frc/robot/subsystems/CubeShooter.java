// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeShooter extends SubsystemBase {
  /** Creates a new CubeShooter. */
  public CubeShooter() {
  }

  public enum CubeShooterStates {
    CALIBRATE("Calibrate"), // only used to rezero the pivot angle
    RETRACTED("Retracted"), // inside fram perimiter, just off the hard stop
    INTAKE("Intake"), // picking cube up off the floor
    SPIT("Spit"), // eject a cube at the floor level for the hybrid node
    SCORE("Score"); // shooting angle

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
    // This method will be called once per scheduler run
  }
}
