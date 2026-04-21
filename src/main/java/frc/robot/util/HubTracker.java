package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.io.IOException;
// import java.io.OutputStream;
// import java.net.HttpURLConnection;
// import java.net.URL;
// import java.nio.charset.StandardCharsets;
// import java.time.Instant;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;


public class HubTracker {
    /**
     * Returns an {@link Optional} containing the current {@link Shift}.
     * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
     */
    public static Optional<Shift> getCurrentShift() {
        double matchTime = getMatchTime();
        if (matchTime < 0) return Optional.empty();

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.endTime) {
                return Optional.of(shift);
            }
        }
        return Optional.empty();
    }

    /**
     * Returns an {@link Optional} containing the current {@link Time} remaining in the current shift.
     * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
     */
    public static Optional<Time> timeRemainingInCurrentShift() {
        return getCurrentShift().map((shift) -> Seconds.of(shift.endTime - getMatchTime()));
    }

    /**
     * Returns an {@link Optional} containing the next {@link Shift}.
     * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
     */
    public static Optional<Shift> getNextShift() {
        double matchTime = getMatchTime();

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.startTime) {
                return Optional.of(shift);
            }
        }
        return Optional.empty();
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Alliance alliance, Shift shift) {
        Optional<Alliance> autoWinner = getAutoWinner();
        switch (shift.activeType) {
            case BOTH:
                return true;
            case AUTO_WINNER:
                return autoWinner.isPresent() && autoWinner.get() == alliance;
            case AUTO_LOSER:
                return autoWinner.isPresent() && autoWinner.get() != alliance;
            default:
                return false;
        }
    }

    /**
     * Returns whether the hub is active during the current {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Alliance alliance) {
        Optional<Shift> currentShift = getCurrentShift();
        return currentShift.isPresent() && isActive(alliance, currentShift.get());
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the robot's {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Shift shift) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && isActive(alliance.get(), shift);
    }

    /**
     * Returns whether the hub is active during the current {@link Shift} for the robot's {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive() {
        Optional<Shift> currentShift = getCurrentShift();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return currentShift.isPresent() && alliance.isPresent() && isActive(alliance.get(), currentShift.get());
    }

    /**
     * Returns whether the hub is active for the next {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActiveNext(Alliance alliance) {
        Optional<Shift> nextShift = getNextShift();
        return nextShift.isPresent() && isActive(alliance, nextShift.get());
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActiveNext() {
        Optional<Shift> nextShift = getNextShift();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return nextShift.isPresent() && alliance.isPresent() && isActive(alliance.get(), nextShift.get());
    }

    /**
     * Returns the {@link Alliance} that won auto as specified by the FMS/Driver Station's game specific message data.
     * Will return {@link Optional#empty()} if no game message or alliance is available.
     */
    public static Optional<Alliance> getAutoWinner() {
        String msg = DriverStation.getGameSpecificMessage();
        char msgChar = msg.length() > 0 ? msg.charAt(0) : ' ';
        switch (msgChar) {
            case 'B':
                return Optional.of(Alliance.Blue);
            case 'R':
                return Optional.of(Alliance.Red);
            default:
                return Optional.empty();
        }
    }

    /**
     * Counts up from 0 to 160 seconds as match progresses.
     * Returns -1 if not match isn't running or if in between auto and teleop
     */
    public static double getMatchTime() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 20 - DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 160 - DriverStation.getMatchTime();
        }
        return -1;
    }

    /**
     * Represents an alliance shift.<br>
     * <h4>Values:</h4>
     * <ul>
     * <li>{@link Shift#AUTO}</li> (0-20 sec)
     * <li>{@link Shift#TRANSITION}</li> (20-30 sec)
     * <li>{@link Shift#SHIFT_1}</li> (30-55 sec)
     * <li>{@link Shift#SHIFT_2}</li> (55-80 sec)
     * <li>{@link Shift#SHIFT_3}</li> (80-105 sec)
     * <li>{@link Shift#SHIFT_4}</li> (105-130 sec)
     * <li>{@link Shift#ENDGAME}</li> (130-160 sec)
     * </ul>
     */
    public enum Shift {
        AUTO(0, 20, ActiveType.BOTH),
        TRANSITION(20, 30, ActiveType.BOTH),
        SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
        SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
        SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
        SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
        ENDGAME(130, 160, ActiveType.BOTH);

        final int startTime;
        final int endTime;
        final ActiveType activeType;

        private Shift(int startTime, int endTime, ActiveType activeType) {
            this.startTime = startTime;
            this.endTime = endTime;
            this.activeType = activeType;
        }
    }

    private enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }

    private static final ScheduledExecutorService ELASTIC_PUBLISHER = Executors.newSingleThreadScheduledExecutor();

    /**
     * Publish the hub tracker status to SmartDashboard using putNumber/putString/putBoolean.
     * The elasticBaseUrl and index parameters are ignored to preserve compatibility with call sites.
     */
    public static void publishStatusToElastic(@SuppressWarnings("unused") String elasticBaseUrl,
                                              @SuppressWarnings("unused") String index) {
        try {
            Optional<Shift> current = getCurrentShift();
            Optional<Shift> next = getNextShift();
            Optional<Alliance> alliance = DriverStation.getAlliance();
            Optional<Alliance> autoWinner = getAutoWinner();

            double matchTime = getMatchTime();
            Double timeRemaining = current.map(s -> s.endTime - matchTime).orElse(null);

            SmartDashboard.putNumber("HubTracker/match_time", matchTime);
            SmartDashboard.putNumber("HubTracker/time_remaining_in_current_shift",
                    timeRemaining == null ? Double.NaN : timeRemaining);
            SmartDashboard.putString("HubTracker/current_shift", current.map(Enum::name).orElse("null"));
            SmartDashboard.putString("HubTracker/next_shift", next.map(Enum::name).orElse("null"));
            SmartDashboard.putString("HubTracker/alliance", alliance.map(Enum::name).orElse("null"));
            SmartDashboard.putString("HubTracker/auto_winner", autoWinner.map(Enum::name).orElse("null"));
            SmartDashboard.putBoolean("HubTracker/is_active", isActive());
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to publish HubTracker status to SmartDashboard: " + e.getMessage(), false);
        }
    }

    /**
     * Start periodic publishing of hub tracker status to SmartDashboard.
     * Returns a ScheduledFuture you can cancel to stop publishing.
     * Example: startPeriodicPublishing("ignored", "ignored", 5);
     */
    public static ScheduledFuture<?> startPeriodicPublishing(String elasticBaseUrl, String index, long periodSeconds) {
        return ELASTIC_PUBLISHER.scheduleAtFixedRate(() -> {
            try {
                publishStatusToElastic(elasticBaseUrl, index);

                // Also provide a compact human-readable status string
                Optional<Shift> current = getCurrentShift();
                Optional<Shift> next = getNextShift();
                Optional<Alliance> alliance = DriverStation.getAlliance();
                Optional<Alliance> autoWinner = getAutoWinner();
                double matchTime = getMatchTime();
                Double timeRemaining = current.map(s -> s.endTime - matchTime).orElse(null);

                String status = String.format("t=%.2f cur=%s next=%s rem=%s ally=%s auto=%s active=%s",
                        matchTime,
                        current.map(Enum::name).orElse("null"),
                        next.map(Enum::name).orElse("null"),
                        timeRemaining == null ? "null" : String.format("%.2f", timeRemaining),
                        alliance.map(Enum::name).orElse("null"),
                        autoWinner.map(Enum::name).orElse("null"),
                        isActive());
                SmartDashboard.putString("HubTracker/status", status);
            } catch (Exception e) {
                DriverStation.reportWarning("Failed to publish HubTracker status to SmartDashboard: " + e.getMessage(), false);
            }
        }, 0, Math.max(1, periodSeconds), TimeUnit.SECONDS);
    }

    /**
     * Stop the publisher thread. Cancels all future periodic publishes.
     */
    public static void stopPeriodicPublishing() {
        ELASTIC_PUBLISHER.shutdownNow();
    }
}
