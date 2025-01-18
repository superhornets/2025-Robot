package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsytem {
    SparkMax m_leftMotor = new SparkMax(ShooterConstants.kMotorLeftCanId, MotorType.kBrushless);
    SparkMax m_rightMotor = new SparkMax(ShooterConstants.kMotorRightCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    private double goal = Double.NaN;

    public ShooterSubsytem() {
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(1000);
        config.encoder.velocityConversionFactor(1000);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(1.0, 0.0, 0.0);
        //m_leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        /*    m_leftMotor.setInverted(ShooterConstants.kIsLeftMotorInverted);
        m_rightMotor.setInverted(ShooterConstants.kIsRightMotorInverted);
        
        m_leftPIDController.setP(ShooterConstants.kShooterP);
        m_leftPIDController.setI(ShooterConstants.kShooterI);
        m_leftPIDController.setD(ShooterConstants.kShooterD);
        m_leftPIDController.setFF(ShooterConstants.kShooterFF);
        m_leftPIDController.setOutputRange(ShooterConstants.kShooterMin, ShooterConstants.kShooterMax);
        m_rightPIDController.setP(ShooterConstants.kShooterP);
        m_rightPIDController.setI(ShooterConstants.kShooterI);
        m_rightPIDController.setD(ShooterConstants.kShooterD);
        m_rightPIDController.setFF(ShooterConstants.kShooterFF);
        m_rightPIDController.setOutputRange(ShooterConstants.kShooterMin, ShooterConstants.kShooterMax);
        
        */
 }
 /*  public void runShooterSubwoofer() {
     m_leftPIDController.setReference(ShooterConstants.kShooterSpeedSubwoofer, ControlType.kVelocity);
     m_rightPIDController.setReference(ShooterConstants.kShooterSpeedSubwoofer, ControlType.kVelocity);
     goal = ShooterConstants.kShooterSpeedSubwoofer;
 }
 
 public void runShooterPodium() {
     m_leftPIDController.setReference(ShooterConstants.kShooterSpeedPodium, ControlType.kVelocity);
     m_rightPIDController.setReference(ShooterConstants.kShooterSpeedPodium, ControlType.kVelocity);
     goal = ShooterConstants.kShooterSpeedPodium;
 }
 
 public void runShooterAmp() {
     m_leftPIDController.setReference(ShooterConstants.kShooterSpeedAmp, ControlType.kVelocity);
     m_rightPIDController.setReference(ShooterConstants.kShooterSpeedAmp, ControlType.kVelocity);
     goal = ShooterConstants.kShooterSpeedAmp;
 }
 
 public void stopShooter() {
     m_leftPIDController.setReference(0, ControlType.kVelocity);
     m_rightPIDController.setReference(0, ControlType.kVelocity);
     goal = Double.NaN;
 }
 
 public void runShooterToInput(double speed) {
     m_leftPIDController.setReference(speed, ControlType.kVelocity);
     m_rightPIDController.setReference(speed, ControlType.kVelocity);
 }
 
 public boolean isAtSpeed() {
     double lowerBound = goal - 300;
     //double upperBound = goal + 300;
 
     return ((m_leftEncoder.getVelocity() > lowerBound) /*&& (m_leftEncoder.getVelocity() < upperBound)*/
        /*&& ((m_rightEncoder.getVelocity() > lowerBound) && (m_rightEncoder.getVelocity() < upperBound))*/;
        /*  @Override
        public void periodic() {
            SmartDashboard.putNumber("velocity", m_leftEncoder.getVelocity());
            SmartDashboard.putBoolean("is at speed", isAtSpeed());
            SmartDashboard.putNumber("Shooter left Voltage", m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage());
            SmartDashboard.putNumber("Shooter left output current", m_leftMotor.getOutputCurrent());
            SmartDashboard.putNumber("Shooter right Voltage",
                    m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage());
            SmartDashboard.putNumber("Shooter right output current", m_rightMotor.getOutputCurrent());
        
        }
        */
}


