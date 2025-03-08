package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax shooter = new SparkMax(CoralConstants.kShootCoralCanId, MotorType.kBrushless);
    private final SparkMax intake = new SparkMax(CoralConstants.kIntakeCoralCanId, MotorType.kBrushless);
    private final SparkLimitSwitch limitA = shooter.getReverseLimitSwitch();
    private final SparkLimitSwitch limitB = shooter.getForwardLimitSwitch();
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();

    public CoralSubsystem() {
        shooterConfig.inverted(true);
        shooterConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchEnabled(false);
        shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.setDefaultCommand(new RunCommand(() -> {
        }, this));

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(false);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
    }

    public void set(double value) {
        shooter.set(value);
        intake.set(value);
        //System.out.println("aAAAAAaaaaaaAA")
        shooterConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        shooter.configureAsync(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {

    }

    public boolean limitAvalue() {
        return limitA.isPressed();
    }

    public boolean limitBvalue() {
        return limitB.isPressed();
    }

    public void intakeCoral() {
        shooterConfig.limitSwitch.reverseLimitSwitchEnabled(limitB.isPressed());
        shooter.configureAsync(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
