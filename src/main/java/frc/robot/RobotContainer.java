// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// teaching

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.AimAtFerry;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Configs.ShooterSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ControlAllShooting;
import frc.robot.commands.ControllAllPassing;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Instantiate Subsystems
  private final Intake m_intake = new Intake();
  private final Belts m_belts = new Belts();
  private final Shooter m_shooter = new Shooter();
  private final Kicker m_kicker = new Kicker();
  private final Pushout m_pushout = new Pushout();

  // Helper Subsystems
  @SuppressWarnings("unused")
  private final HubTrackerSubsystem m_hubtracker = new HubTrackerSubsystem(drivebase, driverXbox);

  // Factory for ControlAllShooting instances. Create a fresh instance for each
  // composition to avoid WPILib's "composed commands may not be reused" error.
  private ControlAllShooting makeVariableShoot() {
    return new ControlAllShooting(drivebase::getCachedDynamicHubLocation, m_shooter, drivebase::getPose);
  }

  private ControllAllPassing makeVariablePass() {
    return new ControllAllPassing(drivebase::getDynamicFerryLocation,
        m_shooter, drivebase::getPose);
  }

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> loggedAutoChooser;
  // Add this field at the top of RobotContainer (alongside your other fields)
  private SendableChooser<Boolean> flipChooser = new SendableChooser<>();

  // -----------------------------------------------------------------------
  // SwerveInputStreams — built in configureBindings() so they reference
  // whichever controller was selected as driver via dc().
  // -----------------------------------------------------------------------

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity;

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle;

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented;

  SwerveInputStream driveAngularVelocityKeyboard;
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard;

  private AimAtHub aimAtHub;
  private AimAtFerry aimAtFerry;

  SwerveInputStream aimAtHubStream;
  SwerveInputStream aimAtFerryStream;

  /** Returns the controller that should be treated as the driving controller. */
  private CommandXboxController dc() {
      return driverXbox;
  }

  private CommandXboxController oc() {
      return operatorXbox;
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // configureFuelSim();
    // configureFuelSimRobot();
    // Triggers for auto aim/pass poses

    DriverStation.silenceJoystickConnectionWarning(true);
    SmartDashboard.putNumber("Heading Bias Deg", 0.0);
    SmartDashboard.putBoolean("Is Shooter Running", m_shooter.isShooterRunning());
    // Tunable gain: radians of bias -> radians/sec of angular velocity
    SmartDashboard.putNumber("Heading Bias Gain", 0);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // // pushout
    NamedCommands.registerCommand("extend", m_pushout.PushCommand());
    NamedCommands.registerCommand("extend and intake",
        Commands.parallel(m_pushout.PushCommand(), m_intake.runIntakeCommand()).withTimeout(4));
    NamedCommands.registerCommand("retract intake", m_pushout.RetractCommand().withTimeout(4));

    // shooter
    NamedCommands.registerCommand("Control All Shooting", Commands.defer(() -> {
      ControlAllShooting shootCmd = new ControlAllShooting(drivebase::getCachedDynamicHubLocation, m_shooter,
          drivebase::getPose, true);
      return Commands.sequence(
          Commands.runOnce(() -> {
            drivebase.setAimLocations();
            drivebase.isAiming = true;
          }),
          Commands.parallel(
              shootCmd,
              drivebase.driveFieldOriented(aimAtHubStream),
              Commands.sequence(
                  Commands.waitUntil(() -> shootCmd.isCASAtSpeed()
                      && aimAtHubStream.aimLock(Angle.ofBaseUnits(1, Degrees)).getAsBoolean()),
                  Commands.parallel(
                      m_belts.RunBeltsCommand(),
                      m_kicker.kickCommand(),
                      m_pushout.CheeksyAgitationCommand(),
                      m_intake.runIntakeCommand()))
                  .finallyDo(() -> Commands.parallel(
                    m_shooter.setTargetRPMCommand(shootCmd.RecordedidealHorizontalSpeed).withTimeout(1),
                    m_pushout.RetractCommand()
                  ))))
          .finallyDo(() -> drivebase.isAiming = false);
    }, java.util.Collections.emptySet()).withTimeout(5.3));

    NamedCommands.registerCommand("Shoot Depot Fuel", Commands.defer(() -> {
      ControlAllShooting shootCmd = new ControlAllShooting(drivebase::getCachedDynamicHubLocation, m_shooter,
          drivebase::getPose, true);
      return Commands.sequence(
          Commands.runOnce(() -> {
            drivebase.setAimLocations();
            drivebase.isAiming = true;
          }),
          Commands.parallel(
              shootCmd,
              drivebase.driveFieldOriented(aimAtHubStream),
              Commands.sequence(
                  Commands.waitUntil(() -> shootCmd.isCASAtSpeed()
                      && aimAtHubStream.aimLock(Angle.ofBaseUnits(1, Degrees)).getAsBoolean()),
                  Commands.parallel(
                      m_belts.RunBeltsCommand(),
                      m_kicker.kickCommand(),
                      m_pushout.CheeksyAgitationCommand(),
                      m_intake.runIntakeCommand()))
                  .finallyDo(() -> Commands.parallel(
                    m_shooter.setTargetRPMCommand(shootCmd.RecordedidealHorizontalSpeed).withTimeout(1),
                    m_pushout.RetractCommand()
                  ))))
          .finallyDo(() -> drivebase.isAiming = false);
    }, java.util.Collections.emptySet()).withTimeout(4));

    
  NamedCommands.registerCommand("Speed Up", m_shooter.SpeedUpShooterCommand());
    

    NamedCommands.registerCommand("SOTM", Commands.defer(() -> {
      ControlAllShooting shootCmd = new ControlAllShooting(drivebase::getCachedDynamicHubLocation, m_shooter,
          drivebase::getPose, true);
      return Commands.sequence(
          Commands.runOnce(() -> {
            drivebase.setAimLocations();
            drivebase.isAiming = true;
            drivebase.shouldAimAtHubAuto = true;
          }),
          Commands.parallel(
              shootCmd,
              // drivebase.driveFieldOriented(aimAtHubStream),
              Commands.sequence(
                  Commands.waitUntil(() -> shootCmd.isCASAtSpeed()
                      // && aimAtHubStream.aimLock(Angle.ofBaseUnits(1, Degrees)).getAsBoolean()
                    ),
                  Commands.parallel(
                      m_belts.RunBeltsCommand(),
                      m_kicker.kickCommand(),
                      // m_pushout.CheeksyAgitationCommand(),
                      m_intake.runIntakeCommand()))
                  .finallyDo(() -> Commands.parallel(
                    m_shooter.setTargetRPMCommand(shootCmd.RecordedidealHorizontalSpeed).withTimeout(1),
                    m_pushout.RetractCommand()
                  ))))
          .finallyDo(() -> {drivebase.isAiming = false; drivebase.shouldAimAtHubAuto = false;});
    }, java.util.Collections.emptySet()).withTimeout(5.3));

    NamedCommands.registerCommand("Shoot Preload", Commands.defer(() -> {
      ControlAllShooting shootCmd = new ControlAllShooting(drivebase::getCachedDynamicHubLocation, m_shooter,
          drivebase::getPose, true);
      return Commands.sequence(
          Commands.runOnce(() -> {
            drivebase.setAimLocations();
            drivebase.isAiming = true;
          }),
          Commands.parallel(
              shootCmd,
              drivebase.driveFieldOriented(aimAtHubStream),
              Commands.sequence(
                  Commands.waitUntil(() -> shootCmd.isCASAtSpeed()
                      && aimAtHubStream.aimLock(Angle.ofBaseUnits(1, Degrees)).getAsBoolean()),
                  Commands.parallel(
                      m_belts.RunBeltsCommand(),
                      m_kicker.kickCommand(),
                      m_pushout.CheeksyAgitationCommand(),
                      m_intake.runIntakeCommand()))
                  .finallyDo(() -> Commands.parallel(
                    m_shooter.setTargetRPMCommand(shootCmd.RecordedidealHorizontalSpeed).withTimeout(1),
                    m_pushout.RetractCommand()
                  ))))
          .finallyDo(() -> {drivebase.isAiming = false; drivebase.shouldAimAtHubAuto = false;});
    }, java.util.Collections.emptySet()).withTimeout(2));

    NamedCommands.registerCommand("intake", m_intake.runIntakeCommand());
    NamedCommands.registerCommand("outtake", m_intake.runOuttakeCommand().withTimeout(4));

    // setup the flip chooser
    flipChooser.setDefaultOption("Not Flipped", false);
    flipChooser.addOption("Flipped", true);
    SmartDashboard.putData("Flip Auto", flipChooser);

    flipChooser.onChange((Boolean flip) -> {
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
          autoStream -> autoStream.map(auto -> {
            auto = new PathPlannerAuto(auto.getName(), flip);
            return auto;
          }));
      autoChooser.setDefaultOption("Do Nothing", Commands.none());
      SmartDashboard.putData("Auto Chooser", autoChooser);
      loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
    });

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        autoStream -> autoStream.map(auto -> {
          auto = new PathPlannerAuto(auto.getName(), flipChooser.getSelected());
          return auto;
        }));
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
  }

  /**
   * Constructs throwaway instances of the commands that fire from deferred RT
   * bindings
   * so first-use class loading (WPILib units system, InterpolatingDoubleTreeMap,
   * SwerveInputStream.copy, command composition) happens at robot boot instead of
   * mid-match. Nothing is scheduled — zero runtime side effects. Side-effect-free
   * because ControlAllShooting's no-arg-requireShooter overload skips
   * addRequirements,
   * and the command constructors only assign fields / copy the input stream.
   */

  public void warmupCommands() {
    @SuppressWarnings("unused")
    ControlAllShooting shootWarm = makeVariableShoot();
    @SuppressWarnings("unused")
    ControllAllPassing passWarm = makeVariablePass();
    @SuppressWarnings("unused")
    AimAtHub aimHubWarm = new AimAtHub(drivebase, driveAngularVelocity,
        dc()::getLeftX, dc()::getLeftY, dc()::getRightX);
    @SuppressWarnings("unused")
    AimAtFerry aimFerryWarm = new AimAtFerry(drivebase, driveAngularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    // Build all SwerveInputStreams here using dc() so they reference the
    // correct driver controller based on the chooser selection.
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> dc().getLeftY() * -1,
        () -> dc().getLeftX() * -1)
        .withControllerRotationAxis(() -> dc().getRightX() * -1)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(1.0)
        .allianceRelativeControl(true);

    dc().rightTrigger().whileTrue(Commands.defer(() -> {
      if (isInAllianceZone()) {
        aimAtHub = new AimAtHub(drivebase, driveAngularVelocity,
            dc()::getLeftX, dc()::getLeftY, dc()::getRightX);
        return aimAtHub;
      } else {
        aimAtFerry = new AimAtFerry(drivebase, driveAngularVelocity);
        return aimAtFerry;
      }
    }, Set.of(drivebase)));

    driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(dc()::getRightX, dc()::getRightY)
        .headingWhile(true);

    driveRobotOriented = driveAngularVelocity.copy()
        .robotRelative(true)
        .allianceRelativeControl(false);

    driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -dc().getLeftY(),
        () -> -dc().getLeftX())
        .withControllerRotationAxis(() -> dc().getRawAxis(2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // Derive the heading axis with math!
    driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(
            () -> Math.sin(dc().getRawAxis(2) * Math.PI) * (Math.PI * 2),
            () -> Math.cos(dc().getRawAxis(2) * Math.PI) * (Math.PI * 2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees(0));

    aimAtHubStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> 0.0, () -> 0.0)
        .withControllerRotationAxis(() -> 0.0)
        .aim(() -> drivebase.getCachedDynamicHubLocation())
        .aimWhile(true)
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds))
        .aimFeedforward(0.0001, 0.0001, 0.00013)
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true);

    aimAtFerryStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> 0.0, () -> 0.0)
        .withControllerRotationAxis(() -> 0.0)
        .aim(() -> drivebase.getCachedDynamicFerryLocation())
        .aimWhile(true)
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds))
        .aimFeedforward(0.0001, 0.0001, 0.00013)
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true);


    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
        () -> applyHeadingBias(driveAngularVelocity.get()));
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(
        () -> applyHeadingBias(driveDirectAngleKeyboard.get()));

    // ======= Driver =======
    dc().rightTrigger().whileTrue(

        Commands.defer(() -> {
          if (isInAllianceZone()) // In alliance zone → shoot at hub
          {
            ControlAllShooting shootCmd = makeVariableShoot();
            return Commands.parallel(
                shootCmd,
                Commands.sequence(
                    Commands.waitUntil(() -> shootCmd.isCASAtSpeed()
                        && aimAtHub.swerveInputStream
                            .aimLock(Angle.ofBaseUnits(aimTolerance(shootCmd.distance), Degrees)).getAsBoolean()),
                    Commands.parallel(
                        Commands.sequence(
                            // Commands.runOnce(() -> Logger.recordOutput(
                            //     "Aim/ShotParallelStartedAt", Timer.getFPGATimestamp())),
                            Commands.waitSeconds(1.0),
                            Commands.runOnce(() -> {
                              aimAtHub.readyToLock = true;
                              Logger.recordOutput("Aim/DynamicAimLockTolerance", aimTolerance(shootCmd.distance));
                              // Logger.recordOutput("Aim/ReadyToLockFiredAt", Timer.getFPGATimestamp());
                            })),
                        m_belts.RunBeltsCommand(),
                        m_kicker.kickCommand(),
                       m_pushout.CheeksyAgitationCommand()
                            // .onlyWhile(() -> !LT_Intake.getAsBoolean())
                            // .beforeStarting(Commands.waitSeconds(1.75)),
                            ,
                        m_intake.runIntakeCommand())
                        .finallyDo(
                            () -> {
                              m_shooter.setTargetRPMCommand(shootCmd.RecordedidealHorizontalSpeed).withTimeout(1);
                              aimAtHub.readyToLock = false;
                            })
                        .onlyWhile(() -> aimAtHub.swerveInputStream
                            .aimLock(Angle.ofBaseUnits(aimTolerance(shootCmd.distance), Degrees)).getAsBoolean())));
          } else {
            ControllAllPassing passCmd = makeVariablePass();
            return Commands.parallel(
                passCmd,
                // Commands.runOnce(() -> driveAngularVelocity.aim(() ->
                // drivebase.getDynamicFerryLocation())),
                Commands.sequence(
                    Commands.waitUntil(() -> passCmd.isCASAtSpeed()
                        && aimAtFerry.swerveInputStream.aimLock(Angle.ofBaseUnits(3, Degrees)).getAsBoolean()),
                    Commands.parallel(
                        m_belts.RunBeltsCommand(),

                        m_kicker.kickCommand(),
                        m_pushout.CheeksyAgitationCommand()
                            .beforeStarting(Commands.waitSeconds(1.5)),
                        m_intake.runIntakeCommand())
                        .onlyWhile(aimAtFerry.swerveInputStream.aimLock(Angle.ofBaseUnits(5, Degrees)))))
                .finallyDo(() -> m_shooter.setTargetRPMCommand(passCmd.RecordedidealHorizontalSpeed).withTimeout(1));
          }
        }, java.util.Collections.emptySet()));
        

    // Intake
    dc().leftTrigger().whileTrue(Commands.parallel(m_pushout.PushCommand(), m_intake.runIntakeCommand()));
    dc().leftBumper().whileTrue(Commands.parallel(m_pushout.RetractCommand(), m_intake.stopIntakeCommand()));

    // Drive to Pose
    dc().povLeft().whileTrue(drivebase.driveToPoseDeffered());

    // Swerve Drive Commands
    dc().start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // ======== Operator ========
    // shooter
    oc().rightTrigger().whileTrue(
        Commands.defer(() -> {
          ControlAllShooting shootCmd = makeVariableShoot();
          return Commands.parallel(
              shootCmd,
              Commands.sequence(
                  Commands.waitUntil(() -> shootCmd.isCASAtSpeed()),
                  Commands.parallel(
                      m_belts.RunBeltsCommand(),
                      m_kicker.kickCommand(),
                      m_pushout.CheeksyAgitationCommand()
                          .beforeStarting(Commands.waitSeconds(1.5)),
                      m_intake.runIntakeCommand()))
                  .finallyDo(
                      () -> m_shooter.setTargetRPMCommand(shootCmd.RecordedidealHorizontalSpeed).withTimeout(1)));
        }, java.util.Collections.emptySet()));

    oc().leftTrigger().whileTrue(
        Commands.parallel(
            // keep running the VariableShoot command while we wait for the shooter to reaal
            // speed
            m_shooter.shootFuelCommand(),

            // once at speed, run hopper + kicker
            Commands.sequence(
                Commands.waitUntil(m_shooter::isShooterFast),
                Commands.parallel(
                    m_belts.RunBeltsCommand(),
                    m_intake.runIntakeCommand(),
                    m_kicker.kickCommand(),
                    // drivebase.lockCommand(
                    // driverXbox::getLeftX,
                    // driverXbox::getLeftY,
                    // driverXbox::getRightX,
                    // driveAngularVelocity::get),
                    m_pushout.CheeksyAgitationCommand()
                        .beforeStarting(Commands.waitSeconds(1.5))))));

    // get to shooter
    oc().rightBumper().whileTrue(
        Commands.parallel(
            m_shooter.ShooterPassingCommand(),

            Commands.sequence(
                Commands.waitUntil(m_shooter::isShooterRunning),
                Commands.parallel(
                    m_belts.RunBeltsCommand(),
                    m_intake.runIntakeCommand(),
                    m_kicker.kickCommand(),
                    m_pushout.CheeksyAgitationCommand()
                        .beforeStarting(Commands.waitSeconds(1.5))))));

    oc().leftBumper().whileTrue(Commands.parallel(m_belts.RunBeltsReverseCommand(), m_kicker.kickBackwardsCommand()));

    oc().start().onTrue(m_pushout.ResetEncoderCommand());

    // intake
    oc().x().whileTrue(m_intake.runIntakeCommand());
    oc().a().whileTrue(m_intake.runOuttakeCommand());

    // pushout
    oc().y().whileTrue(m_pushout.PushoutDutycyleCommand());
    oc().b().whileTrue(m_pushout.PushoutDutycyleRetractCommand());

    // vision
    oc().povUp().onTrue(drivebase.FrontToggle());
    oc().povLeft().onTrue(drivebase.LeftToggle());
    oc().povRight().onTrue(drivebase.VisionToggle());
    oc().povDown().onTrue(drivebase.BackToggle());

    // ========================

    // new Trigger(() -> isInAllianceZone()
    //     && DriverStation.isTeleop())
    //     .onTrue(Commands.runOnce(() -> m_shooter.setDefaultCommand(m_shooter.setAllianceIdle())));
    // new Trigger(() -> !isInAllianceZone()
    //     && DriverStation.isTeleopEnabled())
    //     .onTrue(Commands.runOnce(() -> m_shooter.setDefaultCommand(m_shooter.setNeutralIdle())));
   //  m_shooter.setDefaultCommand(m_shooter.setAllianceIdle().onlyWhile(() -> DriverStation.isTeleopEnabled()));

    // m_intake.setDefaultCommand(m_intake.runDefaultCommand());
    // m_kicker.setDefaultCommand(m_kicker.runDefaultCommand());
    // // m_pushout.setDefaultCommand(m_pushout.runDefaultCommand());
    // m_hopper.setDefaultCommand(m_hopper.runDefaultCommand());

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      if (Constants.USE_ROBOT_RELATIVE) {
        drivebase.setDefaultCommand(
            drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
      } else {
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        // m_shooter.setDefaultCommand(m_shooter.SpeedUpShooterCommand());
      }
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      dc().start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      dc().button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      dc().button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      
      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    if (DriverStation.isTest()) {
      if (Constants.USE_ROBOT_RELATIVE) {
        drivebase.setDefaultCommand(
            drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
      } else {
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides
        // drive command above!
      }
    }

    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    // driverXbox.leftBumper().onTrue(Commands.none());
    // driverXbox.rightBumper().onTrue(Commands.none());
    // } else
    // {
    // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.start().whileTrue(Commands.none());
    // driverXbox.back().whileTrue(Commands.none());
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    // driverXbox.rightBumper().onTrue(Commands.none());
    // }
  }

  private double aimTolerance(double distance) {
    if (distance < 2)
      return 5.0;
    else if (distance < 3.5)
      return 2.0;
    return 1.0;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    Command selected = loggedAutoChooser.get();
    if (selected == null)
      return Commands.none();

    // String selectedName = loggedAutoChooser.get().getName();

    // put the main path (swipe) and the recovery path
    // if (selectedName.equals("Swipe Correction Test")) {
    // return Commands.sequence(
    // followWithRecovery("LT Swipe", "Through LT"),
    // makeAutoShootCommand());
    // }

    return selected;
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void setUseMegaTag2(boolean use) {
    drivebase.useMegaTag2 = use;
  }

  public void logControllerInputs() {
    // Driver left stick X (-1..1).
    Logger.recordOutput("Input/Driver/LeftX", driverXbox.getLeftX());
    // Driver left stick Y (-1..1).
    Logger.recordOutput("Input/Driver/LeftY", driverXbox.getLeftY());
    // Driver right stick X (-1..1).
    Logger.recordOutput("Input/Driver/RightX", driverXbox.getRightX());
    // Driver right stick Y (-1..1).
    Logger.recordOutput("Input/Driver/RightY", driverXbox.getRightY());
    // Driver left trigger (0..1).
    Logger.recordOutput("Input/Driver/LeftTrigger", driverXbox.getLeftTriggerAxis());
    // Driver right trigger (0..1).
    Logger.recordOutput("Input/Driver/RightTrigger", driverXbox.getRightTriggerAxis());

    // Operator left stick X (-1..1).
    Logger.recordOutput("Input/Operator/LeftX", operatorXbox.getLeftX());
    // Operator left stick Y (-1..1).
    Logger.recordOutput("Input/Operator/LeftY", operatorXbox.getLeftY());
    // Operator right stick X (-1..1).
    Logger.recordOutput("Input/Operator/RightX", operatorXbox.getRightX());
    // Operator right stick Y (-1..1).
    Logger.recordOutput("Input/Operator/RightY", operatorXbox.getRightY());
    // Operator left trigger (0..1).
    Logger.recordOutput("Input/Operator/LeftTrigger", operatorXbox.getLeftTriggerAxis());
    // Operator right trigger (0..1).
    Logger.recordOutput("Input/Operator/RightTrigger", operatorXbox.getRightTriggerAxis());

    // --- Shooting sequence state ---
    boolean rtHeld = dc().rightTrigger().getAsBoolean();
    Logger.recordOutput("Shooting/RTHeld", rtHeld);
    Logger.recordOutput("Shooting/InAllianceZone", isInAllianceZone());

    if (aimAtHub != null) {
      Logger.recordOutput("Shooting/AimLock1Deg",
          aimAtHub.swerveInputStream.aimLock(Degrees.of(1.0)).getAsBoolean());
      Logger.recordOutput("Shooting/AimLock3Deg",
          aimAtHub.swerveInputStream.aimLock(Degrees.of(3.0)).getAsBoolean());
    }
    if (aimAtFerry != null) {
      Logger.recordOutput("Shooting/FerryAimLock3Deg",
          aimAtFerry.swerveInputStream.aimLock(Degrees.of(3.0)).getAsBoolean());
    }
  }

  private ChassisSpeeds applyHeadingBias(ChassisSpeeds speeds) {
    // Toggle to enable heading bias; false means pass-through.
    boolean headingBiasEnabled = SmartDashboard.getBoolean("headingBiasEnabled", false);
    if (!headingBiasEnabled) {
      return speeds;
    }
    // Requested heading bias in degrees; 0 means disabled.
    double biasDeg = SmartDashboard.getNumber("Heading Bias Deg", 0.0);
    // Gain mapping bias radians -> added omega (rad/sec).
    double gain = SmartDashboard.getNumber("Heading Bias Gain", 0.0);

    // Default to normal driving (no bias).
    double omega = speeds.omegaRadiansPerSecond;
    if (biasDeg != 0.0 && gain != 0.0) {
      // Convert degrees to radians, then scale into an omega offset.
      double biasRad = Units.degreesToRadians(biasDeg);
      double additionalOmega = gain * biasRad;
      // Leave vx/vy alone; only add a small angular velocity component.
      omega += additionalOmega;
    }

    return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }
}