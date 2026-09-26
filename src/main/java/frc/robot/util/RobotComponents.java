package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
     * Solved from the model: at 145 deg its lowest edge sits 14 mm off the floor, having swept
     * around the front bumper without touching it. Past about 148 deg it digs into the floor.
     */
    public static final double SLAPDOWN_DEPLOYED_DEGREES = 145.0;

    private static final double SLAPDOWN_EXTEND_ROTATIONS = 25.0;

    /**
     * The moving hopper slides straight forward as the slapdown goes out, and stays out to
     * agitate even after the slapdown comes back in - so its extension latches at the furthest
     * the slapdown has reached. Travel measured on the robot by David. The model turned out to
     * be exported RETRACTED, not extended: the other reading put the stowed hopper behind the
     * frame rail, and this one has it stowed inside the frame and reaching 69 mm past the
     * deployed slapdown, which is where he says it sits.
     */
    public static final double HOPPER_TRAVEL_M = Units.inchesToMeters(13.7);

    private static final double SLAPDOWN_ROTATIONS_TO_PUSH_HOPPER = 15.0;

    private static double hopperExtension = 0.0;

    private RobotComponents() {}

    public static void log(Hood hood, Slapdown slapdown) {
        Logger.recordOutput("Components", poses(hood.getAngleDegrees(), slapdown.getPosition()));
        // NOT "Components/..." - a value and a folder at the same path makes AdvantageScope
        // show Components as a folder, and the pose array stops being draggable onto the robot.
        Logger.recordOutput("RobotComponents/HopperExtension", hopperExtension);
    }

    /**
     * How far out the hopper is, 0 to 1. The slapdown drives it out and it latches there, so
     * this only ever goes up until {@link #reset()}.
     */
    public static double hopperExtension(double slapdownRotations) {
        hopperExtension = Math.max(hopperExtension,
                MathUtil.clamp(slapdownRotations / SLAPDOWN_ROTATIONS_TO_PUSH_HOPPER, 0.0, 1.0));
        return hopperExtension;
    }

    /** Forgets that the hopper was pushed out. For tests, and for a fresh run of the sim. */
    public static void reset() {
        hopperExtension = 0.0;
    }

    /**
     * The component poses for a given hood angle in degrees and slapdown position in rotations.
     * This advances the hopper latch, so it is the call every consumer should use.
     */
    public static Pose3d[] poses(double hoodDegrees, double slapdownRotations) {
        return poses(hoodDegrees, slapdownRotations, hopperExtension(slapdownRotations));
    }

    public static Pose3d[] poses(double hoodDegrees, double slapdownRotations, double hopperOut) {
        double hoodPitch = Math.toRadians(HOOD_DIRECTION
                * (hoodDegrees - HoodConstants.rotationsToDegrees(HoodConstants.HOOD_MIN)));
        double slapdownPitch = Math.toRadians(SLAPDOWN_DEPLOYED_DEGREES
                * slapdownRotations / SLAPDOWN_EXTEND_ROTATIONS);
        return new Pose3d[] {
            new Pose3d(HOOD_PIVOT, new Rotation3d(0.0, hoodPitch, 0.0)),
            new Pose3d(SLAPDOWN_PIVOT, new Rotation3d(0.0, slapdownPitch, 0.0)),
            new Pose3d(new Translation3d(HOPPER_TRAVEL_M * hopperOut, 0.0, 0.0),
                    new Rotation3d()),
        };
    }
}
