// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import frc.robot.Commands.ClimberDownCommand;
import frc.robot.Commands.ClimberUpCommand;
import frc.robot.Commands.DriveResetYaw;
import frc.robot.Commands.ElevatorL1Command;
import frc.robot.Commands.ElevatorL2Command;
import frc.robot.Commands.ElevatorL3Command;
import frc.robot.Commands.ElevatorL4Command;
import frc.robot.Commands.ShootCoral;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem(ClimberConstants.kMotorCanId, false);
    private final ElevatorSubsytem m_elevator = new ElevatorSubsytem(ElevatorConstants.kRightMotorCanId,
            ElevatorConstants.kLeftMotorCanId, false);
    private final ExampleSubsystem mExampleSubsystem = new ExampleSubsystem();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //add the auto commands here
        NamedCommands.registerCommand("Shoot", new ShootCoral(m_coralSubsystem));
        NamedCommands.registerCommand("Level 1", new ElevatorL1Command(m_elevator));
        NamedCommands.registerCommand("Level 2", new ElevatorL2Command(m_elevator));
        NamedCommands.registerCommand("Level 3", new ElevatorL3Command(m_elevator));
        NamedCommands.registerCommand("Level 4", new ElevatorL4Command(m_elevator));
        // Configure the button bindings
        //configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Trigger robotRelative = m_driverController.leftTrigger();
        Trigger slowMode = m_driverController.rightTrigger();
        Trigger fastMode = m_driverController.rightBumper();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
        () -> m_robotDrive.teleOpDrive(
                                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                  !robotRelative.getAsBoolean(), true,
                                  slowMode.getAsBoolean(),
                                  fastMode.getAsBoolean()),
                          m_robotDrive));

          //coral intake + de-algifier
          //m_operatorController.leftBumper().whileTrue();

        //climber
        //m_operatorController.povUp().whileTrue(new ClimberUpCommand(m_climber));
        //m_operatorController.povDown().whileTrue(new ClimberDownCommand(m_climber));

       //shooter
        //m_operatorController.rightBumper().whileTrue(new ShootCoral(m_coralSubsystem));

        // NavX
        m_driverController.b().onTrue(new DriveResetYaw(m_robotDrive));

        //elevator
        //L1
        //m_operatorController.a().onTrue();
        //L2
        //m_operatorController.b().onTrue();
        //L3
        //m_operatorController.y().onTrue();
        //L4
        //m_operatorController.x().onTrue();

        //algae
        //Intake
        //m_driverController.leftBumper().onTrue();
        //Arm
        //m_driverController.povUp().whileTrue();
        //m_driverController.povUp().whileTrue();

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    /*private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
      .whileTrue(new RunCommand(
          () -> m_robotDrive.setX(),
          m_robotDrive));
    }*/

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
    
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    } */
}
