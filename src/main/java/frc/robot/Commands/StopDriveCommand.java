package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class StopDriveCommand extends Command {
    private final DriveSubsystem m_subsystem;

    public StopDriveCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_subsystem.drive(0,0,0,true,true);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
