package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

// ouu shi
public class DriveToPoseReplan extends Command {

    private static final double DEFAULT_REPLAN_THRESHOLD_METERS = 0.15;
    private static final double DEFAULT_GOAL_TOLERANCE_METERS = 0.05;
    private static final double DEFAULT_GOAL_TOLERANCE_RADIANS = Units.degreesToRadians(3.0);
    private static final double DEFAULT_MIN_REPLAN_INTERVAL_SECONDS = 0.25;

    private final SwerveSubsystem swerve;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final PathConstraints constraints;
    private final double replanThresholdMeters;
    private final double goalToleranceMeters;
    private final double goalToleranceRadians;
    private final double minReplanIntervalSeconds;

    private Command activePathCommand;
    private final Timer replanCooldown = new Timer();
    private boolean cooldownStarted = false;

    /**
     * Full constructor.
     *
     * @param swerve                  SwerveSubsystem to control.
     * @param targetPoseSupplier      Supplier for the target pose (evaluated on each replan).
     * @param constraints             PathPlanner kinematic constraints.
     * @param replanThresholdMeters   Path-following error that triggers a replan.
     * @param goalToleranceMeters     Translation tolerance to declare the command done.
     * @param goalToleranceRadians    Rotation tolerance to declare the command done.
     * @param minReplanIntervalSeconds Minimum seconds between consecutive replans.
     */
    public DriveToPoseReplan(
            SwerveSubsystem swerve,
            Supplier<Pose2d> targetPoseSupplier,
            PathConstraints constraints,
            double replanThresholdMeters,
            double goalToleranceMeters,
            double goalToleranceRadians,
            double minReplanIntervalSeconds) {
        this.swerve = swerve;
        this.targetPoseSupplier = targetPoseSupplier;
        this.constraints = constraints;
        this.replanThresholdMeters = replanThresholdMeters;
        this.goalToleranceMeters = goalToleranceMeters;
        this.goalToleranceRadians = goalToleranceRadians;
        this.minReplanIntervalSeconds = minReplanIntervalSeconds;
        addRequirements(swerve);
    }

    /** Convenience constructor with a fixed target pose and default tolerances. */
    public DriveToPoseReplan(SwerveSubsystem swerve, Pose2d targetPose, PathConstraints constraints) {
        this(
                swerve,
                () -> targetPose,
                constraints,
                DEFAULT_REPLAN_THRESHOLD_METERS,
                DEFAULT_GOAL_TOLERANCE_METERS,
                DEFAULT_GOAL_TOLERANCE_RADIANS,
                DEFAULT_MIN_REPLAN_INTERVAL_SECONDS);
    }

    /**
     * Convenience constructor using the same default constraints as {@code SwerveSubsystem.driveToPose()}.
     */
    public DriveToPoseReplan(SwerveSubsystem swerve, Pose2d targetPose) {
        this(
                swerve,
                targetPose,
                new PathConstraints(
                        3.5,
                        3.5,
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(540)));
    }

    @Override
    public void initialize() {
        replanCooldown.reset();
        replanCooldown.start();
        cooldownStarted = true;
        startNewPath();
    }

    @Override
    public void execute() {
        if (activePathCommand == null) {
            // Inner command finished; if we're not at goal yet we were disturbed after arrival.
            if (!atGoal()) {
                maybeReplan();
            }
            return;
        }

        // Run the active path command for this loop iteration.
        activePathCommand.execute();

        if (activePathCommand.isFinished()) {
            activePathCommand.end(false);
            activePathCommand = null;
            return;
        }

        // Replan if the robot has drifted too far from the planned trajectory.
        if (swerve.isOffPath(replanThresholdMeters)) {
            maybeReplan();
        }
    }

    @Override
    public boolean isFinished() {
        return activePathCommand == null && atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        if (activePathCommand != null) {
            activePathCommand.end(true);
            activePathCommand = null;
        }
        swerve.stop();
    }

    // ---- private helpers ----

    private void maybeReplan() {
        if (!cooldownStarted || replanCooldown.hasElapsed(minReplanIntervalSeconds)) {
            if (activePathCommand != null) {
                activePathCommand.end(true);
            }
            startNewPath();
            replanCooldown.reset();
        }
    }

    private void startNewPath() {
        activePathCommand = AutoBuilder.pathfindToPose(
                targetPoseSupplier.get(),
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
        activePathCommand.initialize();
    }

    private boolean atGoal() {
        Pose2d current = swerve.getPose();
        Pose2d target = targetPoseSupplier.get();
        double translationError = current.getTranslation().getDistance(target.getTranslation());
        double headingError = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());
        return translationError <= goalToleranceMeters && headingError <= goalToleranceRadians;
    }
}
