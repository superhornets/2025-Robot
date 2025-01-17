package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private SparkLimitSwitch m_limitSwitch;
    //private SparkLimitSwitch m_limitSwitchForward;

    public ClimberSubsystem(int canId, boolean isInverted) {
        // Initialize anything else that couldn't be initialized yetzz
        m_motor = new SparkMax(canId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_limitSwitch = m_motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        //m_limitSwitchForward = m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kMaxHeight);
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);

        // Configure anything
        m_motor.setInverted(isInverted);
        m_encoder.setPositionConversionFactor(ClimberConstants.kEncoderDistancePerRevolution);
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public boolean isDown() {
        return m_limitSwitch.isPressed();
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        if (m_limitSwitch.isPressed()) {
            m_encoder.setPosition(0);

        }

        String label = "motor value" + m_motor.getDeviceId();
        if (m_motor.getDeviceId() == ClimberConstants.kMotorLeftCanId) {
            label = "Left climbed height (Inches)";
        } else if (m_motor.getDeviceId() == ClimberConstants.kMotorRightCanId) {
            label = "Right climbed height (Inches)";
        }
        SmartDashboard.putNumber(label, m_encoder.getPosition());

        SmartDashboard.putNumber("Climber Voltage", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
        SmartDashboard.putNumber("Climber output current", m_motor.getOutputCurrent());
    }
}

}
