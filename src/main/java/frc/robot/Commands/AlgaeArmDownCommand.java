package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeArmDownCommand extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;

    public AlgaeArmDownCommand(AlgaeSubsystem subsystem) {
        addRequirements(subsystem);
        m_algaeSubsystem = subsystem;

    }

    @Override
    public void initialize() {
        m_algaeSubsystem.setArmPosition(AlgaeConstants.ArmSetpoints.kDown);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
