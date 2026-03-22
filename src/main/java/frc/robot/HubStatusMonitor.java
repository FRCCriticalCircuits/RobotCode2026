package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubStatusMonitor {
    private static final String DASHBOARD_PREFIX = "Elastic/";

    private static final double AUTO_LENGTH_SECONDS = 20.0;
    private static final double TRANSITION_END_TIME_SECONDS = 130.0;
    private static final double TRANSITION_LENGTH_SECONDS = 10.0;
    private static final double SHIFT_1_END_TIME_SECONDS = 105.0;
    private static final double SHIFT_2_END_TIME_SECONDS = 80.0;
    private static final double SHIFT_3_END_TIME_SECONDS = 55.0;
    private static final double SHIFT_4_END_TIME_SECONDS = 30.0;
    private static final double SHIFT_LENGTH_SECONDS = 25.0;
    private static final double ENDGAME_LENGTH_SECONDS = 30.0;

    private static final double WARNING_WINDOW_SECONDS = 3.0;
    private static final double WARNING_BLINK_PERIOD_SECONDS = 0.25;
    private static final double WARNING_RUMBLE_PERIOD_SECONDS = 0.40;
    private static final double WARNING_RUMBLE_ON_SECONDS = 0.15;
    private static final double WARNING_RUMBLE_STRENGTH = 0.65;

    private static final double ACTIVE_RUMBLE_DURATION_SECONDS = 0.60;
    private static final double ACTIVE_RUMBLE_STRENGTH = 1.0;

    private final GenericHID driverController;

    private boolean lastOurHubActive = false;
    private double activeRumbleUntilSeconds = 0.0;

    public HubStatusMonitor(GenericHID driverController) {
        this.driverController = driverController;
        publish(HubStatusSnapshot.idle(), false, false);
    }

    public void update() {
        double nowSeconds = Timer.getFPGATimestamp();
        HubStatusSnapshot snapshot = HubStatusSnapshot.fromDriverStation();
        boolean teleopEnabled = DriverStation.isTeleopEnabled();
        boolean hubBecameActive = teleopEnabled && snapshot.ourHubActive() && !lastOurHubActive;

        if (hubBecameActive) {
            activeRumbleUntilSeconds = nowSeconds + ACTIVE_RUMBLE_DURATION_SECONDS;
        }

        boolean warningActive = teleopEnabled
            && snapshot.ourHubActive()
            && snapshot.timeUntilOurHubInactiveSeconds() > 0.0
            && snapshot.timeUntilOurHubInactiveSeconds() <= WARNING_WINDOW_SECONDS;

        boolean displayedHubActive = snapshot.ourHubActive();
        if (warningActive) {
            displayedHubActive =
                ((int) Math.floor(nowSeconds / WARNING_BLINK_PERIOD_SECONDS)) % 2 == 0;
        }

        publish(snapshot, displayedHubActive, warningActive);
        applyRumble(nowSeconds, teleopEnabled, warningActive);

        lastOurHubActive = snapshot.ourHubActive();
    }

    private void applyRumble(double nowSeconds, boolean teleopEnabled, boolean warningActive) {
        double rumbleStrength = 0.0;

        if (teleopEnabled && nowSeconds < activeRumbleUntilSeconds) {
            rumbleStrength = ACTIVE_RUMBLE_STRENGTH;
        } else if (
            warningActive
                && (nowSeconds % WARNING_RUMBLE_PERIOD_SECONDS) < WARNING_RUMBLE_ON_SECONDS
        ) {
            rumbleStrength = WARNING_RUMBLE_STRENGTH;
        }

        driverController.setRumble(RumbleType.kBothRumble, rumbleStrength);
    }

    private void publish(
        HubStatusSnapshot snapshot,
        boolean displayedHubActive,
        boolean warningActive
    ) {
        SmartDashboard.putString(DASHBOARD_PREFIX + "Hub Phase", snapshot.phaseName());
        SmartDashboard.putNumber(
            DASHBOARD_PREFIX + "Hub Countdown (s)",
            snapshot.phaseCountdownSeconds()
        );
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Red Hub Active", snapshot.redHubActive());
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Blue Hub Active", snapshot.blueHubActive());
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Our Hub Active", displayedHubActive);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Our Hub Active Raw", snapshot.ourHubActive());
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Hub Warning Active", warningActive);
        SmartDashboard.putNumber(
            DASHBOARD_PREFIX + "Time Until Our Hub Inactive (s)",
            finiteDashboardNumber(snapshot.timeUntilOurHubInactiveSeconds())
        );
    }

    private static double finiteDashboardNumber(double value) {
        return Double.isFinite(value) ? value : -1.0;
    }

    private record HubStatusSnapshot(
        String phaseName,
        double phaseCountdownSeconds,
        boolean redHubActive,
        boolean blueHubActive,
        boolean ourHubActive,
        double timeUntilOurHubInactiveSeconds
    ) {
        private static HubStatusSnapshot idle() {
            return new HubStatusSnapshot("Disabled", 0.0, false, false, false, 0.0);
        }

        private static HubStatusSnapshot fromDriverStation() {
            Optional<Alliance> alliance = DriverStation.getAlliance();

            if (DriverStation.isAutonomousEnabled()) {
                return new HubStatusSnapshot(
                    "Auto",
                    clampCountdown(DriverStation.getMatchTime(), AUTO_LENGTH_SECONDS),
                    true,
                    true,
                    alliance.isPresent(),
                    Double.POSITIVE_INFINITY
                );
            }

            if (!DriverStation.isTeleopEnabled()) {
                return idle();
            }

            double matchTimeSeconds = Math.max(DriverStation.getMatchTime(), 0.0);
            String gameData = DriverStation.getGameSpecificMessage();
            Boolean redInactiveFirst = parseRedInactiveFirst(gameData);

            if (redInactiveFirst == null) {
                return new HubStatusSnapshot(
                    matchTimeSeconds > TRANSITION_END_TIME_SECONDS
                        ? "Hub Transition"
                        : "Teleop (No Hub Data)",
                    phaseCountdownForTeleop(matchTimeSeconds),
                    true,
                    true,
                    alliance.isPresent(),
                    Double.POSITIVE_INFINITY
                );
            }

            boolean redShiftOneActive = !redInactiveFirst;
            boolean blueShiftOneActive = redInactiveFirst;

            boolean redHubActive;
            boolean blueHubActive;
            String phaseName;
            double phaseCountdownSeconds;

            if (matchTimeSeconds > TRANSITION_END_TIME_SECONDS) {
                phaseName = "Hub Transition";
                phaseCountdownSeconds = clampCountdown(
                    matchTimeSeconds - TRANSITION_END_TIME_SECONDS,
                    TRANSITION_LENGTH_SECONDS
                );
                redHubActive = true;
                blueHubActive = true;
            } else if (matchTimeSeconds > SHIFT_1_END_TIME_SECONDS) {
                phaseName = "Shift 1";
                phaseCountdownSeconds = clampCountdown(
                    matchTimeSeconds - SHIFT_1_END_TIME_SECONDS,
                    SHIFT_LENGTH_SECONDS
                );
                redHubActive = redShiftOneActive;
                blueHubActive = blueShiftOneActive;
            } else if (matchTimeSeconds > SHIFT_2_END_TIME_SECONDS) {
                phaseName = "Shift 2";
                phaseCountdownSeconds = clampCountdown(
                    matchTimeSeconds - SHIFT_2_END_TIME_SECONDS,
                    SHIFT_LENGTH_SECONDS
                );
                redHubActive = !redShiftOneActive;
                blueHubActive = !blueShiftOneActive;
            } else if (matchTimeSeconds > SHIFT_3_END_TIME_SECONDS) {
                phaseName = "Shift 3";
                phaseCountdownSeconds = clampCountdown(
                    matchTimeSeconds - SHIFT_3_END_TIME_SECONDS,
                    SHIFT_LENGTH_SECONDS
                );
                redHubActive = redShiftOneActive;
                blueHubActive = blueShiftOneActive;
            } else if (matchTimeSeconds > SHIFT_4_END_TIME_SECONDS) {
                phaseName = "Shift 4";
                phaseCountdownSeconds = clampCountdown(
                    matchTimeSeconds - SHIFT_4_END_TIME_SECONDS,
                    SHIFT_LENGTH_SECONDS
                );
                redHubActive = !redShiftOneActive;
                blueHubActive = !blueShiftOneActive;
            } else {
                phaseName = "Endgame";
                phaseCountdownSeconds = clampCountdown(matchTimeSeconds, ENDGAME_LENGTH_SECONDS);
                redHubActive = true;
                blueHubActive = true;
            }

            boolean ourHubActive = alliance
                .map(
                    allianceColor -> allianceColor == Alliance.Red ? redHubActive : blueHubActive
                )
                .orElse(false);

            double timeUntilOurHubInactiveSeconds = alliance
                .map(
                    allianceColor ->
                        timeUntilInactive(
                            allianceColor,
                            matchTimeSeconds,
                            redInactiveFirst.booleanValue()
                        )
                )
                .orElse(Double.POSITIVE_INFINITY);

            return new HubStatusSnapshot(
                phaseName,
                phaseCountdownSeconds,
                redHubActive,
                blueHubActive,
                ourHubActive,
                timeUntilOurHubInactiveSeconds
            );
        }

        private static Boolean parseRedInactiveFirst(String gameData) {
            if (gameData.isEmpty()) {
                return null;
            }

            return switch (gameData.charAt(0)) {
                case 'R' -> true;
                case 'B' -> false;
                default -> null;
            };
        }

        private static double phaseCountdownForTeleop(double matchTimeSeconds) {
            if (matchTimeSeconds > TRANSITION_END_TIME_SECONDS) {
                return clampCountdown(
                    matchTimeSeconds - TRANSITION_END_TIME_SECONDS,
                    TRANSITION_LENGTH_SECONDS
                );
            }

            if (matchTimeSeconds > SHIFT_1_END_TIME_SECONDS) {
                return clampCountdown(matchTimeSeconds - SHIFT_1_END_TIME_SECONDS, SHIFT_LENGTH_SECONDS);
            }

            if (matchTimeSeconds > SHIFT_2_END_TIME_SECONDS) {
                return clampCountdown(matchTimeSeconds - SHIFT_2_END_TIME_SECONDS, SHIFT_LENGTH_SECONDS);
            }

            if (matchTimeSeconds > SHIFT_3_END_TIME_SECONDS) {
                return clampCountdown(matchTimeSeconds - SHIFT_3_END_TIME_SECONDS, SHIFT_LENGTH_SECONDS);
            }

            if (matchTimeSeconds > SHIFT_4_END_TIME_SECONDS) {
                return clampCountdown(matchTimeSeconds - SHIFT_4_END_TIME_SECONDS, SHIFT_LENGTH_SECONDS);
            }

            return clampCountdown(matchTimeSeconds, ENDGAME_LENGTH_SECONDS);
        }

        private static double timeUntilInactive(
            Alliance alliance,
            double matchTimeSeconds,
            boolean redInactiveFirst
        ) {
            boolean shiftOneActive = alliance == Alliance.Red ? !redInactiveFirst : redInactiveFirst;

            if (matchTimeSeconds > TRANSITION_END_TIME_SECONDS) {
                return shiftOneActive
                    ? matchTimeSeconds - SHIFT_1_END_TIME_SECONDS
                    : matchTimeSeconds - TRANSITION_END_TIME_SECONDS;
            }

            if (matchTimeSeconds > SHIFT_1_END_TIME_SECONDS) {
                return shiftOneActive
                    ? matchTimeSeconds - SHIFT_1_END_TIME_SECONDS
                    : Double.POSITIVE_INFINITY;
            }

            if (matchTimeSeconds > SHIFT_2_END_TIME_SECONDS) {
                return shiftOneActive
                    ? Double.POSITIVE_INFINITY
                    : matchTimeSeconds - SHIFT_2_END_TIME_SECONDS;
            }

            if (matchTimeSeconds > SHIFT_3_END_TIME_SECONDS) {
                return shiftOneActive
                    ? matchTimeSeconds - SHIFT_3_END_TIME_SECONDS
                    : Double.POSITIVE_INFINITY;
            }

            return Double.POSITIVE_INFINITY;
        }

        private static double clampCountdown(double value, double maxValue) {
            if (Double.isNaN(value) || Double.isInfinite(value)) {
                return maxValue;
            }

            return Math.max(0.0, Math.min(value, maxValue));
        }
    }
}
