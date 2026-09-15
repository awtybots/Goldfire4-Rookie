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
    private double aimErrorDegrees = 180.0;

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

    private double aimTolerance(double dist) {
        return 1.0;
    }

    private double aimErrorTo(Translation2d target, Translation2d robotPos) {
        double bearing = target.minus(robotPos).getAngle()
                .minus(drivebase.getPose().getRotation()).getDegrees();
        return Math.abs(MathUtil.inputModulus(bearing - 180.0, -180.0, 180.0));
    }

    private boolean isReadyToFire() {
        return inShootingZone
                && isAtSpeed
                && m_hood.isAtPosition()
                && aimErrorDegrees <= aimTolerance(distance);
    }

    @Override
    public void initialize() {
        isFiring = false;
        isAtSpeed = false;
        aimErrorDegrees = 180.0;
    }

    @Override
    public void execute() {
        Translation2d robotPos = drivebase.getPose().getTranslation();
        inShootingZone = !drivebase.isInOpponentAllianceZone();

        if (drivebase.isInAllianceZone()) { // shoot at hub
            Translation2d robotToHub = drivebase.getCachedDynamicHubLocation()
                    .getTranslation().minus(robotPos);
            double dist = robotToHub.getNorm();
            distance = dist;
            aimErrorDegrees = aimErrorTo(
                    drivebase.getCachedDynamicHubLocation().getTranslation(), robotPos);

            double targetRPM = ShooterConstants.hubShooterTable.get(MathUtil.clamp(dist,
                    ShooterConstants.MIN_HUB_DISTANCE_M, ShooterConstants.MAX_HUB_DISTANCE_M));
            recordedTargetRPM = targetRPM;

            m_shooter.setTargetRPM(targetRPM);
            isAtSpeed = Math.abs(m_shooter.getRPM() - targetRPM)
                    <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Hub");
            Logger.recordOutput("Shooting/DistanceToHub", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else if (drivebase.isInNeutralZone()) { // ferry
            Translation2d robotToFerry = drivebase.getCachedDynamicFerryLocation()
                    .getTranslation().minus(robotPos);
            double dist = robotToFerry.getNorm();
            distance = dist;
            aimErrorDegrees = aimErrorTo(
                    drivebase.getCachedDynamicFerryLocation().getTranslation(), robotPos);

            double targetRPM = ShooterConstants.ferryShooterTable.get(MathUtil.clamp(dist,
                    ShooterConstants.MIN_FERRY_DISTANCE_M, ShooterConstants.MAX_FERRY_DISTANCE_M));
            recordedTargetRPM = targetRPM;

            m_shooter.setTargetRPM(targetRPM);
            isAtSpeed = Math.abs(m_shooter.getRPM() - targetRPM)
                    <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Ferry");
            Logger.recordOutput("Shooting/DistanceToFerry", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else { // opponent alliance zone - we never shoot or ferry from here
            recordedTargetRPM = ShooterConstants.ALLIANCE_IDLE_RPM;
            m_shooter.setTargetRPM(ShooterConstants.ALLIANCE_IDLE_RPM);
            isAtSpeed = false;
            aimErrorDegrees = 180.0;
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
        Logger.recordOutput("Shooting/IsFiring", isFiring);
        Logger.recordOutput("Shooting/IsReadyToFire", isReadyToFire());
        Logger.recordOutput("Shooting/AimErrorDeg", aimErrorDegrees);
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
