package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class DeAlgifyCommand extends Command {
    private final CoralSubsystem m_coralSubsystem;

    public DeAlgifyCommand(CoralSubsystem subsystem) {
        m_coralSubsystem = subsystem;
        addRequirements(subsystem);
        //System.out.println("a");
    }

    @Override
    public void initialize() {
        m_coralSubsystem.set(-0.2);
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
        return false;
    }

}
