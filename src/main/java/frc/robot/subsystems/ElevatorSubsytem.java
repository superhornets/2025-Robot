package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {
    private final SparkMax m_motor1 = new SparkMax(ElevatorConstants.kRightMotorCanId, MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(ElevatorConstants.kLeftMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor1.getEncoder();
    private final SparkClosedLoopController m_ClosedLoopController1 = m_motor1.getClosedLoopController();
    private final SparkLimitSwitch m_switch = m_motor1.getReverseLimitSwitch();
    //private final SparkClosedLoopController m_ClosedLoopController2 = m_motor2.getClosedLoopController();
    //private double goal = Double.NaN;
    
    public ElevatorSubsytem(int canId1, int canId2, boolean isInverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD)
                .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
        m_motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(false)
                .follow(m_motor1, true);
        config2.closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD)
                .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
        m_motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
    }

     public double getPosition() {
         return m_encoder.getPosition();
     }

     public void set(double speed) {
         m_motor1.set(speed);
         //m_ClosedLoopController1.setReference(speed, SparkBase.ControlType.kPosition);

    }

    public void moveTo(double level) {
        //System.out.println("Hello");
        //m_ClosedLoopController1.setReference(level, SparkBase.ControlType.kPosition);
        //m_ClosedLoopController2.setReference(level, SparkBase.ControlType.kPosition);
        m_ClosedLoopController1.setReference(level, SparkBase.ControlType.kPosition);
        //m_ClosedLoopController2.setReference(level, SparkBase.ControlType.kPosition);
        //goal = level;
    }

    /*public void holdPosition() {
        //m_ClosedLoopController1.setReference(goal, ControlType.kPosition);
        //m_ClosedLoopController2.setReference(goal, ControlType.kPosition);
    }*/

     @Override
    public void periodic() {
        if (m_switch.isPressed() == true) {
            m_encoder.setPosition(0);
        }

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        //String label = "motor value" + m_motor1.getDeviceId();
        //String label2 = "motor value" + m_motor1.getDeviceId();
        SmartDashboard.putNumber("moter value" + m_motor1.getDeviceId(), m_encoder.getPosition());
        //SmartDashboard.putNumber(label2, m_encoder.getPosition());

        SmartDashboard.putNumber("Elevator Motor 1 applied output", m_motor1.getAppliedOutput());
        SmartDashboard.putNumber("Elevator Motor 2 applied output", m_motor2.getAppliedOutput());
        SmartDashboard.putNumber("Elevator Motor 1 output current", m_motor1.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Motor 2 output current", m_motor2.getOutputCurrent());
    }
}
