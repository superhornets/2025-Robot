package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsytem;

public class ElevtorL4Command extends Command {
    //Declare subsystem variables
    private final ElevatorSubsytem m_elevatorSubsystem;

    public ElevtorL4Command(ElevatorSubsytem elevatorSubsytem){
        addRequirements(elevatorSubsytem);
        m_elevatorSubsystem = elevatorSubsytem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_elevatorSubsystem.moveTo(ElevatorConstants.kL4);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return m_elevatorSubsystem.isAtSetpoint();
    }
}
