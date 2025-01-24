package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.SimulationRobotConstants;
public class AlgaeArmSubsystem extends SubsystemBase {
    
    private SparkFlex armMotor = new SparkFlex(AlgaeConstants.kPivotMotorCanId, MotorType.kBrushless);
 private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

    private boolean stowWhenIdle = true;
    private boolean wasReset = false;

    private DCMotor armMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim m_intakeSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kIntakeReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kIntakeLength, SimulationRobotConstants.kIntakeMass),
          SimulationRobotConstants.kIntakeLength,
          SimulationRobotConstants.kIntakeMinAngleRads,
          SimulationRobotConstants.kIntakeMaxAngleRads,
          true,
          SimulationRobotConstants.kIntakeMinAngleRads,
          0.0,
          0.0);

private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Ball Intake Root", 28, 3);
  private final MechanismLigament2d intakePivotMechanism =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Intake Pivot",
              SimulationRobotConstants.kIntakeShortBarLength
                  * SimulationRobotConstants.kPixelsPerMeter,
              Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)));

    public class AlgaeSubsystem {
    armMotor.configure(
        Configs.AlgaeSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);




        armEncoder.setPosition(0);
    }
}
