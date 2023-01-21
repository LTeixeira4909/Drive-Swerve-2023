package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    double error;

    public BalanceCommand() {
        m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double pitch;
        
        final double pitch_setpoint = 0;
        // Max charge station angle: 15 degrees
        final double max_angle = 15.0;
        // Max velocity proportion: 0.3
        final double max_vel_prop = 0.3;
        final double kP = max_vel_prop/max_angle;
        double driveRate;
        
        pitch = m_drivetrainSubsystem.getGyroPitch();

        error = pitch - pitch_setpoint;

        driveRate = error * kP;

        if (driveRate > max_vel_prop) {
            driveRate = max_vel_prop;
        }

        m_drivetrainSubsystem.DriveWithVelocity(driveRate, 0, 0);

    }

    
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.DriveWithVelocity(0, 0, 0);
    }
    
    @Override
    public boolean isFinished() {
        final double pitch_threshold = 1;
        return Math.abs(error) < pitch_threshold;
    }
}
