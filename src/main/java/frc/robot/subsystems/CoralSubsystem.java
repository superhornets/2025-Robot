package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax motor1 = new SparkMax(CoralConstants.kCanId, MotorType.kBrushless);
    private final SparkMax motor2 = new SparkMax(CoralConstants.kCanId2, MotorType.kBrushless);

    public CoralSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(false);
        motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
    }

    public void set(double value) {
        motor1.set(value);
        motor2.set(value);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {

    }

}
