package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    
public class CoralSubsystem extends SubsystemBase {
    /** Subsystem-wide setpoints */
    public enum Setpoint {
      kFeederStation,
      kLevel1,
      kLevel2,
      kLevel3,
      kLevel4;
    }
  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor =
      new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();










    public static final double kIntakeReduction = 135; // 135:1
public static final double kIntakeLength = 0.4032262; // m
public static final double kIntakeMass = 5.8738; // Kg
public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
public static final double kIntakeShortBarLength = 0.1524;
public static final double kIntakeLongBarLength = 0.3048;
public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);

}

