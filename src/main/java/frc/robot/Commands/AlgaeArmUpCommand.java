package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeArmUpCommand extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;

    public AlgaeArmUpCommand(AlgaeSubsystem subsystem) {
        addRequirements(subsystem);
        m_algaeSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        m_algaeSubsystem.setIntakePosition(AlgaeConstants.ArmSetpoints.kStow);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
