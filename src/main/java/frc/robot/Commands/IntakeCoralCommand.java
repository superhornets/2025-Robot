package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class IntakeCoralCommand extends Command {
    private final CoralSubsystem m_coralSubsystem;

    public IntakeCoralCommand(CoralSubsystem subsystem) {
        m_coralSubsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_coralSubsystem.limitAvalue()) {
            if (m_coralSubsystem.limitBvalue()) {
                m_coralSubsystem.set(0.1);
            } else {
                m_coralSubsystem.set(0.1);
            }
        } else {
            if (m_coralSubsystem.limitBvalue()) {
                m_coralSubsystem.set(0);
            } else {
                m_coralSubsystem.set(0.1);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_coralSubsystem.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
