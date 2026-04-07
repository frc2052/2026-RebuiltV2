package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

public class MatchState {

  /**
   * Checks if the current alliance is red or blue. If the alliance is not defined, it returns
   * false.
   *
   * @return true if the alliance is red, false if the alliance is blue or not defined.
   */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return (alliance.get() == DriverStation.Alliance.Red);
    } else {
      return false;
    }
  }

  /**
   * Gets the current match time. If the match time is not defined, it returns 0 seconds.
   *
   * @return the current match time in seconds.
   */
  public static Time getMatchTime() {
    Double time = DriverStation.getMatchTime();
    if (time.isNaN()) {
      return Seconds.of(0.0);
    } else {
      return Seconds.of(time);
    }
  }

  /**
   * Checks if the robot is currently in teleoperated mode.
   *
   * @return true if the robot is in teleoperated mode, false otherwise.
   */
  public static boolean isTeleop() {
    return DriverStation.isTeleop();
  }

  /**
   * Checks if the robot is currently in autonomous mode.
   *
   * @return true if the robot is in autonomous mode, false otherwise.
   */
  public static boolean isAutonomous() {
    return DriverStation.isAutonomous();
  }

  /**
   * Checks if the robot is currently in test mode.
   *
   * @return true if the robot is in test mode, false otherwise.
   */
  public static boolean isTest() {
    return DriverStation.isTest();
  }

  /**
   * Checks if the robot is currently disabled.
   *
   * @return true if the robot is disabled, false otherwise.
   */
  public static boolean isDisabled() {
    return DriverStation.isDisabled();
  }

  /**
   * Checks if the robot is currently enabled.
   *
   * @return true if the robot is enabled, false otherwise.
   */
  public static boolean isEnabled() {
    return DriverStation.isEnabled();
  }

  /**
   * Checks if the hub is currently active based on the current shift and alliance color. The hub is
   * active during the autonomous period, transition shift, and endgame. During teleop, the hub is
   * active based on the alliance color and current shift.
   *
   * @return true if the hub is active, false otherwise.
   */
  public static boolean isHubActive() {
    Shift currentShift = getCurrentShift();
    if (currentShift == Shift.AUTO
        || currentShift == Shift.TRANSITION_SHIFT
        || currentShift == Shift.ENDGAME) return true;

    String gameData;
    gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {

      if (RobotState.getInstance().isFirstShiftOverride()) {
        if (gameData.charAt(0) == 'R') {
          gameData = gameData.replaceFirst("R", "B");
        } else if (gameData.charAt(0) == 'B') {
          gameData = gameData.replaceFirst("B", "R");
        }
      }

      switch (gameData.charAt(0)) {
        case 'R': // red hub inactive first, will be active for shift 2 and shift 4
          if (currentShift == Shift.SHIFT_ONE || currentShift == Shift.SHIFT_THREE) {
            return !isRedAlliance();
          } else {
            return isRedAlliance();
          }
        case 'B': // blue hub inactive first, will be active for shift 2 and shift 4
          if (currentShift == Shift.SHIFT_ONE || currentShift == Shift.SHIFT_THREE) {
            return isRedAlliance();
          } else {
            return !isRedAlliance();
          }
        default:
          System.out.println("Unexpected game data: " + gameData);
          if (currentShift == Shift.SHIFT_ONE || currentShift == Shift.SHIFT_THREE) {
            return false;
          } else {
            return true;
          }
      }
    } else {
      // System.out.println("No game Data Received");
      if (currentShift == Shift.SHIFT_ONE || currentShift == Shift.SHIFT_THREE) {
        return false;
      } else {
        return true;
      }
    }
  }

  /**
   * Gets the current shift based on the match time and robot mode. The shift is determined by the
   * match time and the robot mode (autonomous or teleop). If the robot is in autonomous mode, it
   * returns the AUTO shift. If the robot is in teleop mode, it checks the match time against the
   * defined shifts and returns the appropriate shift. If the robot is not in either mode, it
   * returns NOT_DEFINED.
   *
   * @return the current shift.
   */
  public static Shift getCurrentShift() {
    Time time = getMatchTime();
    if (isAutonomous()) {
      return Shift.AUTO;
    } else if (isTeleop()) {
      if (time.in(Seconds) >= Shift.TRANSITION_SHIFT.getEndTime().in(Seconds)) {
        return Shift.TRANSITION_SHIFT;
      } else if (time.in(Seconds) >= Shift.SHIFT_ONE.getEndTime().in(Seconds)) {
        return Shift.SHIFT_ONE;
      } else if (time.in(Seconds) >= Shift.SHIFT_TWO.getEndTime().in(Seconds)) {
        return Shift.SHIFT_TWO;
      } else if (time.in(Seconds) >= Shift.SHIFT_THREE.getEndTime().in(Seconds)) {
        return Shift.SHIFT_THREE;
      } else if (time.in(Seconds) >= Shift.SHIFT_FOUR.getEndTime().in(Seconds)) {
        return Shift.SHIFT_FOUR;
      } else {
        return Shift.ENDGAME;
      }
    } else {
      return Shift.NOT_DEFINED;
    }
  }

  /**
   * Gets the time remaining until the current shift changes. If the robot is in autonomous mode, it
   * returns the current match time. If the robot is in teleop mode, it returns the time remaining
   * until the end of the current shift. If the robot is not in either mode, it returns 0 seconds.
   *
   * @return the time remaining until the current shift changes.
   */
  public static Time getTimeToShiftChange() {
    Time time = getMatchTime();
    if (isAutonomous()) {
      return time;
    } else if (isTeleop()) {
      return Seconds.of(time.in(Seconds) - getCurrentShift().getEndTime().in(Seconds));
    } else {
      return Seconds.of(0.0);
    }
  }

  /**
   * Gets the time remaining until the specified shift starts.
   *
   * @param shift the shift to get the time until.
   * @return the time remaining until the specified shift starts.
   */
  public static Time getTimeUntil(Shift shift) {
    Time time = getMatchTime();
    return Seconds.of(shift.getEndTime().in(Seconds) - time.in(Seconds));
  }

  /**
   * Checks if scoring is allowed based on the current shift and hub activity. Scoring is allowed if
   * the hub is active or if the time remaining until the shift changes is less than or equal to 3
   * seconds.
   *
   * @return true if scoring is allowed, false otherwise.
   */
  public static boolean isScoringAllowed() {
    // Shift currentShift = getCurrentShift();
    Time timeToShiftChange = getTimeToShiftChange();
    return isHubActive()
        || (timeToShiftChange.in(Seconds)
            <= 3.0); // allow scoring if hub is active or within last 3 seconds of an inactive shift
    // as it takes time for the fuel to get scored
  }

  public enum Shift {
    AUTO(Seconds.of(20.0), Seconds.of(0.0)),
    TRANSITION_SHIFT(Seconds.of(140.0), Seconds.of(130.0)),
    SHIFT_ONE(Seconds.of(130.0), Seconds.of(105.0)),
    SHIFT_TWO(Seconds.of(105.0), Seconds.of(80.0)),
    SHIFT_THREE(Seconds.of(80.0), Seconds.of(55.0)),
    SHIFT_FOUR(Seconds.of(55.0), Seconds.of(30.0)),
    ENDGAME(Seconds.of(30.0), Seconds.of(0.0)),
    NOT_DEFINED(Seconds.of(0.0), Seconds.of(0.0));

    private Time StartTime;
    private Time EndTime;

    public Time getStartTime() {
      return this.StartTime;
    }

    public Time getEndTime() {
      return this.EndTime;
    }

    public Time getShiftLength() {
      return Seconds.of(this.StartTime.in(Seconds) - this.EndTime.in(Seconds));
    }

    private Shift(Time startTime, Time endTime) {
      this.StartTime = startTime;
      this.EndTime = endTime;
    }
  }
}
