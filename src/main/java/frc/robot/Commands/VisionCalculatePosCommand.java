package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionAprilTagSubsystem;

public class VisionCalculatePosCommand extends Command {
    private final VisionAprilTagSubsystem m_visionAprilTagSubsystem;
    private final DriveSubsystem m_driveSubsystem;

    public VisionCalculatePosCommand(VisionAprilTagSubsystem visionAprilTagSubsystem, DriveSubsystem driveSubsystem) {
        m_visionAprilTagSubsystem = visionAprilTagSubsystem;
        m_driveSubsystem = driveSubsystem;
        addRequirements(visionAprilTagSubsystem);

    }

    @Override
    public void execute() {
        //m_driveSubsystem.odometryAddVisionMeasurement(
        //      m_visionAprilTagSubsystem.getEstimatedGlobalPose(m_driveSubsystem.getPose()));
    }
}
