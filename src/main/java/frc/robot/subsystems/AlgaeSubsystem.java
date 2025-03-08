package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax m_motor1 = new SparkMax(AlgaeConstants.kSparkMotorId, MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(AlgaeConstants.kFlexMotorId, MotorType.kBrushless);

    public AlgaeSubsystem() {
        /*SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        m_motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
        
        SparkFlexConfig config2 = new SparkFlexConfig();
        config2.inverted(false);
        m_motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));*/
    }

    public void setSpark(double value) {
        m_motor1.set(value);
    }

    public void setFlex(double value) {
        m_motor2.set(value);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {

    }
}
