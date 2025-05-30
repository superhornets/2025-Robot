package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ServoSubsystem;

public class ServoOutCommand extends Command {
    private final ServoSubsystem m_subsystem;

    public ServoOutCommand(ServoSubsystem servoSubsystem) {
        addRequirements(servoSubsystem);
        m_subsystem = servoSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_subsystem.set(.8);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
