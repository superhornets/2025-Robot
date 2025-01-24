package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

        @SuppressWarnings("unused")
        private final MechanismLigament2d intakePivotSecondMechanism =
            intakePivotMechanism.append(
                new MechanismLigament2d(
                    "Intake Pivot Second Bar",
                    SimulationRobotConstants.kIntakeLongBarLength
                        * SimulationRobotConstants.kPixelsPerMeter,
                    Units.radiansToDegrees(SimulationRobotConstants.kIntakeBarAngleRads)));

    SmartDashboard.putData("Algae Subsystem", m_mech2d);
    armEncoder.setPosition(0);

    armMotorSim = new SparkFlexSim(armMotor, armMotorModel);
}

/** Zero the arm encoder when the user button is pressed on the roboRIO */
  private void zeroOnUserButton() {
    if (!wasReset && RobotController.getUserButton()) {
      // Zero the encoder only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasReset = true;
      armEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasReset = false;
    }
  }

  public Command stowCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = true;
        });
  }

   @Override
  public void periodic() {
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Algae/Arm/Position", armEncoder.getPosition());

    // Update mechanism2d
    intakePivotMechanism.setAngle(
        Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)
            + Units.rotationsToDegrees(
                armEncoder.getPosition() / SimulationRobotConstants.kIntakeReduction));
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_intakeSim.getCurrentDrawAmps();
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_intakeSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_intakeSim.update(0.020);

    // Iterate the arm SPARK simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_intakeSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
    }

