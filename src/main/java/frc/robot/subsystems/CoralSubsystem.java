package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax motor1 = new SparkMax(CoralConstants.kShootCoralCanId, MotorType.kBrushless);
    private final SparkMax motor2 = new SparkMax(CoralConstants.kIntakeCoralCanId, MotorType.kBrushless);
    private final SparkLimitSwitch limitA = motor1.getReverseLimitSwitch();
    private final SparkLimitSwitch limitB = motor1.getForwardLimitSwitch();

    public CoralSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.limitSwitch.forwardLimitSwitchEnabled(false);
        config.limitSwitch.reverseLimitSwitchEnabled(false);
        motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.setDefaultCommand(new RunCommand(() -> {
        }, this));

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(false);
        motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.setDefaultCommand(new RunCommand(() -> {
        }, this));
    }

    public void set(double value) {
        motor1.set(value);
        motor2.set(value);
        //System.out.println("aAAAAAaaaaaaAA");
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

    public boolean hasCoral() {
        if ((limitAvalue() == true) && (limitBvalue() == true)) {
            return true;
        } else {
            return false;
        }
    }
}
