package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final SparkMax m_motorRight = new SparkMax(IndexerConstants.kMotorRightCanId, MotorType.kBrushless);
    private final SparkMax m_motorLeft = new SparkMax(IndexerConstants.kMotorLeftCanId, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    private final SparkBase m_switchUp = m_motorLeft
            .getForwardLimitSwitch(SparkBase.Type.kNormallyClosed);
    private final SparkBase m_switchDown = m_motorRight
            .getReverseLimitSwitch(SparkBase.Type.kNormallyClosed);

    public IndexerSubsystem() {
        // Initialize anything else that couldn't be initialized yet
        m_switchDown.enableLimitSwitch(false);
        // Configure anything
        m_motorRight.follow(m_motorLeft, true);

        // m_motorRight.setInverted(IndexerConstants.kMotorRightInverted);
        m_motorLeft.setInverted(IndexerConstants.kMotorLeftInverted);

        this.setDefaultCommand(new RunCommand(() -> {
            m_motorLeft.set(0);
        }, this));

    }

    public void intake() {
        if (getNoteAcquired()) {
            m_motorLeft.set(0);
        } else {
            m_motorLeft.set(IndexerConstants.kIntakeSpeed);
        }
    }

    public void reverse() {
        m_motorLeft.set(IndexerConstants.kReverseIntakeSpeed);
    }

    public void shoot() {
        m_motorLeft.set(IndexerConstants.kFeedSpeed);
    }

    public boolean getNoteAcquired() {
        return m_switchUp.isPressed();
    }

    public boolean getNoteRumble() {
        return m_switchDown.isPressed();
    }

    public void stop() {
        m_motorLeft.set(0);
    }

    public void setSwitchEnabled() {
        m_switchUp.enableLimitSwitch(true);
    }

    public void setSwitchDisabled() {
        m_switchUp.enableLimitSwitch(false);
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.
        SmartDashboard.putBoolean("Have Note", getNoteAcquired());
        SmartDashboard.putNumber("Indexer Voltage", m_motorLeft.getAppliedOutput() * m_motorLeft.getBusVoltage());
        SmartDashboard.putNumber("Indexer output current", m_motorLeft.getOutputCurrent());
    }
}
