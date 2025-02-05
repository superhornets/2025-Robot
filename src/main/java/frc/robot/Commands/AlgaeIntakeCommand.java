package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;

    public AlgaeIntakeCommand(AlgaeSubsystem subsystem) {
        m_algaeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_algaeSubsystem.setSpark(0.2);
        m_algaeSubsystem.setFlex(0.1);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_algaeSubsystem.setSpark(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
