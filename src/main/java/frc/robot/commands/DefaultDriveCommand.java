package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    Joystick m_driver;

    public DefaultDriveCommand(Joystick driver) {
        m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
        m_driver = driver;

        this.addRequirements(m_drivetrainSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.DriveWithJoystick(m_driver);
    }

    
    @Override
    public void end(boolean interrupted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
