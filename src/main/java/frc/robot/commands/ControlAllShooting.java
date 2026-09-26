package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ControlAllShooting extends Command {

    private final Shooter m_shooter;
    private final Hopper m_hopper;
    private final Kicker m_kicker;
    private final Hood m_hood;
    private final SwerveSubsystem drivebase;

    public double distance = 0.0;
    public double recordedTargetRPM = 0.0;
    private boolean isFiring = false;
    private boolean isAtSpeed = false;
    private boolean inShootingZone = true; // false in the opponent alliance zone
    private boolean tooClose = false;
    private boolean hubMode = false;
    private double aimErrorDegrees = 180.0;
    private double aimToleranceDegrees = 0.0;

    public ControlAllShooting(Shooter shooter, Hopper hopper, Kicker kicker, Hood hood,
            SwerveSubsystem swerve) {
        this.m_shooter = shooter;
        this.m_hopper = hopper;
        this.m_kicker = kicker;
        this.m_hood = hood;
        this.drivebase = swerve;

        addRequirements(shooter, hopper, kicker); // AimHood does its own thing so we dont need it here
    }

    public boolean isAtSpeed() {
        return isAtSpeed;
    }

    public boolean isFiring() {
        return isFiring;
    }

    private double aimTolerance(double dist, double allowedMissMeters) {
        return Math.toDegrees(Math.atan2(allowedMissMeters, dist));
    }

    private boolean atSpeed(double targetRPM, double startMargin, double keepMargin) {
        double behind = targetRPM - m_shooter.getRPM();
        if (behind <= startMargin) {
            return true; // never block the feed for running fast
        }
        return isFiring && behind <= keepMargin; // ride through the dip each ball causes
    }

    private double aimErrorTo(Translation2d target, Translation2d robotPos) {
        double bearing = target.minus(robotPos).getAngle()
                .minus(drivebase.getPose().getRotation()).getDegrees();
        return Math.abs(MathUtil.inputModulus(bearing - 180.0, -180.0, 180.0));
    }

    private boolean isReadyToFire() {
        // The hub tables sit at the hood floor, which is exactly where the trench lock holds the
        // hood, so a hub shot next to the trench is unaffected. Only ferrying needs the hood up.
        return inShootingZone
                && !tooClose
                && (hubMode || !m_hood.isTrenchLocked())
                && isAtSpeed
                && m_hood.isAtPosition()
                && aimErrorDegrees <= aimToleranceDegrees;
    }

    @Override
    public void initialize() {
        isFiring = false;
        isAtSpeed = false;
        tooClose = false;
        hubMode = false;
        aimErrorDegrees = 180.0;
        aimToleranceDegrees = 0.0;
    }

    @Override
    public void execute() {
        Translation2d robotPos = drivebase.getPose().getTranslation();
        inShootingZone = !drivebase.isInOpponentAllianceZone();

        if (drivebase.isHubShot()) { // shoot at hub
            hubMode = true;
            Translation2d robotToHub = drivebase.getCachedDynamicHubLocation()
                    .getTranslation().minus(robotPos);
            double dist = robotToHub.getNorm();
            distance = dist;
            aimErrorDegrees = aimErrorTo(
                    drivebase.getCachedDynamicHubLocation().getTranslation(), robotPos);
            aimToleranceDegrees = aimTolerance(dist, ShooterConstants.HUB_AIM_TOLERANCE_M);
            tooClose = dist < ShooterConstants.MIN_HUB_SHOT_DISTANCE_M;

            double targetRPM = ShooterConstants.hubShooterTable.get(MathUtil.clamp(dist,
                    ShooterConstants.MIN_HUB_DISTANCE_M, ShooterConstants.MAX_HUB_DISTANCE_M));
            recordedTargetRPM = targetRPM;

            m_shooter.setTargetRPM(targetRPM);
            isAtSpeed = atSpeed(targetRPM, ShooterConstants.ERROR_MARGIN,
                    ShooterConstants.KEEP_FEEDING_MARGIN);

            Logger.recordOutput("Shooting/Mode", "Hub");
            Logger.recordOutput("Shooting/DistanceToHub", dist);
        } else if (drivebase.isInNeutralZone()) { // ferry
            hubMode = false;
            Translation2d robotToFerry = drivebase.getCachedDynamicFerryLocation()
                    .getTranslation().minus(robotPos);
            double dist = robotToFerry.getNorm();
            distance = dist;
            aimErrorDegrees = aimErrorTo(
                    drivebase.getCachedDynamicFerryLocation().getTranslation(), robotPos);
            // a pass only has to land in our zone, so it stays loose enough to shoot on the move
            aimToleranceDegrees = ShooterConstants.FERRY_AIM_TOLERANCE_DEG;
            tooClose = false;

            double targetRPM = ShooterConstants.ferryShooterTable.get(MathUtil.clamp(dist,
                    ShooterConstants.MIN_FERRY_DISTANCE_M, ShooterConstants.MAX_FERRY_DISTANCE_M));
            recordedTargetRPM = targetRPM;

            m_shooter.setTargetRPM(targetRPM);
            isAtSpeed = atSpeed(targetRPM, ShooterConstants.FERRY_ERROR_MARGIN,
                    ShooterConstants.FERRY_KEEP_FEEDING_MARGIN);

            Logger.recordOutput("Shooting/Mode", "Ferry");
            Logger.recordOutput("Shooting/DistanceToFerry", dist);
        } else { // opponent alliance zone - we never shoot or ferry from here
            hubMode = false;
            recordedTargetRPM = ShooterConstants.ALLIANCE_IDLE_RPM;
            m_shooter.setTargetRPM(ShooterConstants.ALLIANCE_IDLE_RPM);
            isAtSpeed = false;
            tooClose = false;
            aimErrorDegrees = 180.0;
            aimToleranceDegrees = 0.0;
            Logger.recordOutput("Shooting/Mode", "HoldOpponentZone");
        }

        if (isReadyToFire()) {
            isFiring = true;
            m_kicker.Kick();
            m_hopper.BeltsToConveyor();
        } else {
            isFiring = false;
            m_hopper.stopBelts();
            m_kicker.stopKicking();
        }

        Logger.recordOutput("Shooting/TargetRPM", recordedTargetRPM);
        Logger.recordOutput("Shooting/CurrentRPM", m_shooter.getRPM());
        Logger.recordOutput("Shooting/IsAtSpeed", isAtSpeed);
        Logger.recordOutput("Shooting/RpmBehind", recordedTargetRPM - m_shooter.getRPM());
        Logger.recordOutput("Shooting/IsFiring", isFiring);
        Logger.recordOutput("Shooting/IsReadyToFire", isReadyToFire());
        Logger.recordOutput("Shooting/AimErrorDeg", aimErrorDegrees);
        Logger.recordOutput("Shooting/AimToleranceDeg", aimToleranceDegrees);
        Logger.recordOutput("Shooting/TooClose", tooClose);
        Logger.recordOutput("Shooting/HoodTrenchLocked", m_hood.isTrenchLocked());
        Logger.recordOutput("Shooting/HoodAtPosition", m_hood.isAtPosition());
        Logger.recordOutput("Shooting/Distance", distance);
        Logger.recordOutput("Shooting/InShootingZone", inShootingZone);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released
    }

    @Override
    public void end(boolean interrupted) {

        m_hopper.stopBelts();
        m_kicker.stopKicking();
        m_shooter.stopShooting();
        isFiring = false;
        isAtSpeed = false;
    }
}
