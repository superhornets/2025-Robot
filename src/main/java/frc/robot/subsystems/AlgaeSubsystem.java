package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Commands.AlgaeIntakeCommand;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.SimulationRobotConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private SparkMax armMotor = new SparkMax(AlgaeConstants.kPivotMotorCanId, MotorType.kBrushless);
    private SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private RelativeEncoder armEncoder = armMotor.getEncoder();
    private SparkLimitSwitch forwardLimitSwitch = armMotor.getForwardLimitSwitch();
    private SparkLimitSwitch reverseLimitSwitch = armMotor.getReverseLimitSwitch();

    private SparkFlex intakeMotor = new SparkFlex(AlgaeConstants.kIntakeMotorCanId, MotorType.kBrushless);

    private boolean stowWhenIdle = true;
    private boolean wasReset = false;

    private DCMotor armMotorModel = DCMotor.getNEO(1);
    private SparkMaxSim armMotorSim;
    private final SingleJointedArmSim m_intakeSim = new SingleJointedArmSim(
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
    private final MechanismLigament2d intakePivotMechanism = m_mech2dRoot.append(
            new MechanismLigament2d(
                    "Intake Pivot",
                    SimulationRobotConstants.kIntakeShortBarLength * SimulationRobotConstants.kPixelsPerMeter,
                    Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)));

    @SuppressWarnings("unused")
    private final MechanismLigament2d intakePivotSecondMechanism = intakePivotMechanism.append(
            new MechanismLigament2d(
                    "Intake Pivot Second Bar",
                    SimulationRobotConstants.kIntakeLongBarLength * SimulationRobotConstants.kPixelsPerMeter,
                    Units.radiansToDegrees(SimulationRobotConstants.kIntakeBarAngleRads)));

    public AlgaeSubsystem() {
        /*
        * Apply the configuration to the SPARKs.
        *
        * kResetSafeParameters is used to get the SPARK to a known state. This
        * is useful in case the SPARK is replaced.
        *
        * kPersistParameters is used to ensure the configuration is not lost when
        * the SPARK loses power. This is useful for power cycles that may occur
        * mid-operation.
        */
        intakeMotor.configure(
                Configs.AlgaeSubsystem.intakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        armMotor.configure(
                Configs.AlgaeSubsystem.armConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SmartDashboard.putData("Algae Subsystem", m_mech2d);

        armEncoder.setPosition(0);

        this.setDefaultCommand(new RunCommand(() -> {

        }, this));

        armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
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

    /*  public Command runIntakeCommand() {
        return this.run(
                () -> {
                    stowWhenIdle = false;
                    setIntakePower(AlgaeConstants.IntakeSetpoints.kForward);
                    setIntakePosition(AlgaeConstants.ArmSetpoints.kDown);
                });
    }
    
    public Command reverseIntakeCommand() {
        return this.run(
                () -> {
                    stowWhenIdle = true;
                    setIntakePower(AlgaeConstants.IntakeSetpoints.kReverse);
                    setIntakePosition(AlgaeConstants.ArmSetpoints.kHold);
                });
    }
    
    public Command stowCommand() {
        return this.runOnce(
                () -> {
                    stowWhenIdle = true;
                });
    }
    
    public Command idleCommand() {
        return this.run(
                () -> {
                    if (stowWhenIdle = true) {
                        setIntakePower(0.0);
                        setIntakePosition(AlgaeConstants.ArmSetpoints.kStow);
                    } else {
                        setIntakePower(AlgaeConstants.IntakeSetpoints.kHold);
                        setIntakePosition(AlgaeConstants.ArmSetpoints.kHold);
                    }
                });
    }
    */
    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public void setArmPosition(double position) {
        armController.setReference(position, ControlType.kPosition);
    }

    public void setArmPower(double power) {
        armMotor.set(power);
    }

    @Override
    public void periodic() {
        zeroOnUserButton();
        if (reverseLimitSwitch.isPressed()) {
            armEncoder.setPosition(0);
        }
        // Display subsystem values
        SmartDashboard.putNumber("Algae/Arm/Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Algae/Intake/Applied Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putBoolean("Algae/Arm/Forward Switch Value", forwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Algae/Arm/Reverse Switch Value", reverseLimitSwitch.isPressed());
        // Update mechanism2d
        intakePivotMechanism.setAngle(
                Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)
                        + Units.rotationsToDegrees(
                                armEncoder.getPosition() / SimulationRobotConstants.kIntakeReduction));
    }

    public double getSimulationCurrentDraw() {
        return m_intakeSim.getCurrentDrawAmps();
    }

    @Override
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
