package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(CoralConstants.kCanId, MotorType.kBrushless);

    public CoralSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.encoder.convertionFactor(20) ;
        motor.configure(config, );
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
    }


    public void set(double value) {
        motor.set(value);
    }

    @Override
    public void periodic() {
    }

}







