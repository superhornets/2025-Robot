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
        m_coralSubsystem.set(0.5);
    }

    @Override
    public void execute() {
        System.out.println("Hellllllllo");
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