package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class ShootCoralCommand extends Command {
    private final CoralSubsystem m_coralSubsystem;

    public ShootCoralCommand(CoralSubsystem subsystem) {
        m_coralSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_coralSubsystem.set(0.3);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_coralSubsystem.set(0);
    }

    @Override
    public boolean isFinished() {
        if (m_coralSubsystem.hasCoral() == false) {
            return true;
        } else {
            return false;
        }
    }

}