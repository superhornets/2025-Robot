package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsytem;

public class IndexerShootCommand extends Command {
    // Declare subsystem variables
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsytem m_shooter;

    private double timeStamp;
    private boolean hasStartedShooting = false;

    public IndexerShootCommand(IndexerSubsystem indexer, ShooterSubsytem shooter) {
        addRequirements(indexer);
        m_indexer = indexer;
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_indexer.setSwitchDisabled();
        timeStamp = Timer.getFPGATimestamp();
        System.out.println("start Shooting");
        hasStartedShooting = false;
    }

    @Override
    public void execute() {
        if (m_shooter.isAtSpeed() || hasStartedShooting) {
            m_indexer.shoot();
            hasStartedShooting = true;
        } else {
            timeStamp = Timer.getFPGATimestamp();
            System.out.println("Not At Speed!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("stop shooting");
        m_indexer.stop();
    }

    @Override
    public boolean isFinished() {
        if ((Timer.getFPGATimestamp() - timeStamp) >= IndexerConstants.kTime) {
            return true;
        }
        return false;
    }
}
