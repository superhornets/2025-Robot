package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveResetYaw extends Command {
    // Declare subsystem variables
    private final DriveSubsystem m_robotDrive;

    public DriveResetYaw(DriveSubsystem robotDrive) {
        addRequirements(robotDrive);
        m_robotDrive = robotDrive;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_robotDrive.resetYaw();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
