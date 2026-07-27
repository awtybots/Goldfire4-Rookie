package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;


public class AimHood extends Command {

    private final Hood hood;
    private final Supplier<Pose2d> goalPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final boolean ferryMode; // false = hub LUT, true = ferry LUT

    public double distance = 0.0;

    private static final InterpolatingDoubleTreeMap hubHoodTable = HoodConstants.hubHoodTable;
    private static final InterpolatingDoubleTreeMap ferryHoodTable = HoodConstants.ferryHoodTable;

    public AimHood(Hood hood, Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> robotPoseSupplier,
            boolean ferryMode) {
        this.hood = hood;
        this.goalPoseSupplier = goalPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.ferryMode = ferryMode;
        addRequirements(hood);
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
