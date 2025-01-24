package frc.robot.subsystems;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.SimulationRobotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;




public class AlgaeIntake {
    
}
intakeMotor.configure(
    Configs.AlgaeSubsystem.intakeConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
armMotor.configure(
    Configs.AlgaeSubsystem.armConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

// Display mechanism2d
SmartDashboard.putData("Algae Subsystem", m_mech2d);

// Zero arm encoder on initialization
armEncoder.setPosition(0);
  // Member variables for subsystem state management
  private boolean stowWhenIdle = true;
  private boolean wasReset = false;
// Initialize Simulation values
armMotorSim = new SparkFlexSim(armMotor, armMotorModel);

 /** Zero the arm encoder when the user button is pressed on the roboRIO */
 private void zeroOnUserButton() {
    if (!wasReset && RobotController.getUserButton()) {
      // Zero the encoder only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasReset = true;

    } else if (!RobotController.getUserButton()) {
      wasReset = false;
    }
  }
    /**
   * Command to run the algae intake. This will extend the arm to its "down" position and run the
   * motor at its "forward" power to intake the ball.
   *
   * <p>This will also update the idle state to hold onto the ball when this command is not running.
   */
  public Command runIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = false;
          setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kForward);
          setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kDown);
        });
  }
    /**
   * Command to run the algae intake in reverse. This will extend the arm to its "hold" position and
   * run the motor at its "reverse" power to eject the ball.
   *
   * <p>This will also update the idle state to stow the arm when this command is not running.
   */
  public Command reverseIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = true;
          setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kReverse);
          setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kHold);
        });
  }

  /** Command to force the subsystem into its "stow" state. */
  public Command stowCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = true;
        });
  }
  /**
   * Command to run when the intake is not actively running. When in the "hold" state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position and stop the
   * motor.
   */
  public Command idleCommand() {
    return this.run(
        () -> {
          if (stowWhenIdle) {
            setIntakePower(0.0);
            setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kStow);
          } else {
            setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kHold);
            setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kHold);
          }
        });
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }




  @Override
  public void periodic() {
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Algae/Arm/Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Algae/Intake/Applied Output", intakeMotor.getAppliedOutput());

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