package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeOuttakeCommand extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;

    public AlgaeOuttakeCommand(AlgaeSubsystem subsystem) {
        m_algaeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_algaeSubsystem.setIntakePower(-0.4);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_algaeSubsystem.setIntakePower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
