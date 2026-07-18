package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.function.Supplier;

/**
 * Continuously sets the hood angle (shot angle) based on distance from the
 * robot to the goal pose, using a distance -> degrees lookup table.
 * Ported from the Offseason-Kraken AimHood command, adapted to the
 * goal/robot pose supplier pattern used by ControlAllShooting since this
 * robot has no turret.
 */
public class AimHood extends Command {

    private final Hood hood;
    private final Supplier<Pose2d> goalPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final boolean ferryMode; // false = hub LUT, true = ferry LUT

    public double distance = 0.0;

    private final InterpolatingDoubleTreeMap hubHoodTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ferryHoodTable = new InterpolatingDoubleTreeMap();

    public AimHood(Hood hood, Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> robotPoseSupplier,
            boolean ferryMode) {
        this.hood = hood;
        this.goalPoseSupplier = goalPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.ferryMode = ferryMode;
        addRequirements(hood);

        // aim at hub LUT: launch angle in degrees, steeper up close, flatter far
        // out. 3m anchored at ~62 deg per the tuning notes in ControlAllShooting.
        // TUNE THIS!!!
        for (var entry : List.of(
                Pair.of(Meters.of(2.0), Degrees.of(70.0)),
                Pair.of(Meters.of(2.5), Degrees.of(66.0)),
                Pair.of(Meters.of(3.0), Degrees.of(62.0)),
                Pair.of(Meters.of(3.5), Degrees.of(58.0)),
                Pair.of(Meters.of(4.0), Degrees.of(54.0)),
                Pair.of(Meters.of(5.0), Degrees.of(49.0)),
                Pair.of(Meters.of(6.0), Degrees.of(45.0)))) {
            hubHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        }

        // aim at ferry LUT: flatter lobs for distance - TUNE THIS!!!
        for (var entry : List.of(
                Pair.of(Meters.of(2.0), Degrees.of(60.0)),
                Pair.of(Meters.of(3.0), Degrees.of(55.0)),
                Pair.of(Meters.of(4.0), Degrees.of(50.0)),
                Pair.of(Meters.of(5.0), Degrees.of(47.0)),
                Pair.of(Meters.of(6.0), Degrees.of(45.0)))) {
            ferryHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        }
    }

    private boolean isUnderTrench() { // checks if robot is under the trench by looking at the field x pos
        double x = robotPoseSupplier.get().getX();
        return Math.abs(x - HoodConstants.TRENCH_X_BLUE) <= HoodConstants.TRENCH_THRESHOLD
                || Math.abs(x - HoodConstants.TRENCH_X_RED) <= HoodConstants.TRENCH_THRESHOLD;
    }

    @Override
    public void execute() {
        if (isUnderTrench()) { // keep hood tucked so it doesn't hit the trench
            hood.setAngle(HoodConstants.HOOD_MIN_DEGREES);
            Logger.recordOutput("Hood/IsUnderTrench", true);
            return;
        }
        Logger.recordOutput("Hood/IsUnderTrench", false);

        Translation2d goalLocation = goalPoseSupplier.get().getTranslation();
        Translation2d robotLocation = robotPoseSupplier.get().getTranslation();
        distance = goalLocation.minus(robotLocation).getNorm();

        double targetAngle = ferryMode ? ferryHoodTable.get(distance) : hubHoodTable.get(distance);
        hood.setAngle(targetAngle);

        Logger.recordOutput("Hood/Mode", ferryMode ? "Ferry" : "Hub");
        Logger.recordOutput("Hood/Distance", distance);
        Logger.recordOutput("Hood/LUTTargetAngle", targetAngle);
        Logger.recordOutput("Hood/CurrentAngle", hood.getAngleDegrees());
    }

    @Override
    public boolean isFinished() {
        return false; // runs while the shooting binding is held
    }

    @Override
    public void end(boolean interrupted) {
        hood.goToMin(); // tuck hood down when command ends
    }
}
