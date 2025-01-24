package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private double goal = Double.NaN;
    
     public ClimberSubsystem(int canId, boolean isInverted){
        m_motor = new SparkMax(canId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

        // Configure anything
        m_motor.setInverted(isInverted);
     }

     public void set(double speed) {
        m_motor.set(speed);
    }

    public void moveTo(double level) {
        goal = level;
    }

    public boolean isAtSetpoint() {
        double upperBound = goal + 4;
        double lowerBound = goal - 4;
        return (m_encoder.getPosition() > lowerBound) && (m_encoder.getPosition() < upperBound);
    }

     @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        String label = "motor value" + m_motor.getDeviceId();
        SmartDashboard.putNumber(label, m_encoder.getPosition());

        SmartDashboard.putNumber("Elevator Voltage", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
        SmartDashboard.putNumber("Elevator output current", m_motor.getOutputCurrent());
    }
}
