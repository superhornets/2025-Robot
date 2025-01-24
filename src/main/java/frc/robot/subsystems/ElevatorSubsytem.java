package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ElevatorSubsytem extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    
     public ElevatorSubsytem(int canId, boolean isInverted){
        m_motor = new SparkMax(canId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

        // Configure anything
        m_motor.setInverted(isInverted);
     }

     public void set(double speed) {
        m_motor.set(speed);
    }

     @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

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
