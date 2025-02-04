package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {
    private final SparkMax m_motor1 = new SparkMax(ElevatorConstants.kRightMotorCanId, MotorType.kBrushless);;
    private final SparkMax m_motor2 = new SparkMax(ElevatorConstants.kLeftMotorCanId, MotorType.kBrushless);;
    //private final RelativeEncoder m_encoder;
    private final AbsoluteEncoder m_encoder = m_motor1.getAbsoluteEncoder();
    private double goal = Double.NaN;
    
    public ElevatorSubsytem(int canId1, int canId2, boolean isInverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        m_motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(false);
        m_motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
     }

     public void set(double speed) {
         m_motor1.set(speed);
         m_motor2.set(speed);
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

        String label = "motor value" + m_motor1.getDeviceId();
        String label2 = "motor value" + m_motor1.getDeviceId();
        SmartDashboard.putNumber(label, m_encoder.getPosition());
        SmartDashboard.putNumber(label2, m_encoder.getPosition());

        SmartDashboard.putNumber("Elevator Motor 1 Voltage", m_motor1.getAppliedOutput() * m_motor1.getBusVoltage());
        SmartDashboard.putNumber("Elevator Motor 2 Voltage", m_motor2.getAppliedOutput() * m_motor2.getBusVoltage());
        SmartDashboard.putNumber("Elevator Motor 1 output current", m_motor1.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Motor 2 output current", m_motor2.getOutputCurrent());
    }
}
