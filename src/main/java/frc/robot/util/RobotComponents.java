package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Slapdown;
import org.littletonrobotics.junction.Logger;

/**
 * Poses for the moving parts of the AdvantageScope robot model, in the order that
 * advantagescope/Robot_Goldfire4/config.json lists them: hood, slapdown, moving hopper.
 *
 * <p>The pivots were measured off the CAD export and are in robot coordinates - x forward, y
 * left, z up, origin in the middle of the frame at floor level. config.json's zeroedPosition
 * already brings each component's pivot to that origin, so a pose here is only where the pivot
 * sits and how far the mechanism has swung from the position it was exported in.
 */
public final class RobotComponents {

    /** Bottom shooter roller axis: the hood swings around it so its belt keeps its length. */
    private static final Translation3d HOOD_PIVOT = new Translation3d(-0.2245, 0.0, 0.4578);

    private static final Translation3d SLAPDOWN_PIVOT = new Translation3d(0.3016, 0.0, 0.2413);

    /** Raising the hood pitches it back and up; the other way drives it into the shooter. */
    private static final double HOOD_DIRECTION = -1.0;

    /**
     * How far the slapdown has swung out once {@link Slapdown#extend()} has run to its setpoint.
     * Every other number here came off the CAD; this one is an estimate of the gear ratio, so it
     * is the one to change if the model deploys too far or not far enough.
     */
    public static final double SLAPDOWN_DEPLOYED_DEGREES = 90.0;

    private static final double SLAPDOWN_EXTEND_ROTATIONS = 25.0;

    private RobotComponents() {}

    public static void log(Hood hood, Slapdown slapdown) {
        Logger.recordOutput("Components", poses(hood.getAngleDegrees(), slapdown.getPosition()));
    }

    /** The component poses for a given hood angle in degrees and slapdown position in rotations. */
    public static Pose3d[] poses(double hoodDegrees, double slapdownRotations) {
        double hoodPitch = Math.toRadians(HOOD_DIRECTION
                * (hoodDegrees - HoodConstants.rotationsToDegrees(HoodConstants.HOOD_MIN)));
        double slapdownPitch = Math.toRadians(SLAPDOWN_DEPLOYED_DEGREES
                * slapdownRotations / SLAPDOWN_EXTEND_ROTATIONS);
        return new Pose3d[] {
            new Pose3d(HOOD_PIVOT, new Rotation3d(0.0, hoodPitch, 0.0)),
            new Pose3d(SLAPDOWN_PIVOT, new Rotation3d(0.0, slapdownPitch, 0.0)),
            new Pose3d(),
        };
    }
}
