package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Balance
 */
public class Balance extends CommandBase {

    BangBangController controller;
    DrivetrainSubsystem dt;
    double m_setpoint;

    public Balance(double setpoint) {
        controller = new BangBangController();
        dt = DrivetrainSubsystem.getInstance();
        m_setpoint = setpoint;
        addRequirements(dt);
    }

    @Override
    public void execute() {
        double output = controller.calculate(dt.getPitch(), m_setpoint);
        
        
        double leftRightDir = output;
        double fwdBackDir = output;
        double turn = output;
        
        dt.go(fwdBackDir, leftRightDir, turn);
    
    
    
    }
}