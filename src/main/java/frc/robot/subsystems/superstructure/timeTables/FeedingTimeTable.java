package frc.robot.subsystems.superstructure.timeTables;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.HashMap;
import java.util.Map;

public class FeedingTimeTable extends TimeTableBase {

  private static FeedingTimeTable INSTANCE;

  static {
    Map<Distance, Time> timeTable = new HashMap<>();
    // Initialize the time table with distance and corresponding time pairs
    timeTable.put(Meters.of(5.28), Seconds.of(1.2));
    timeTable.put(Meters.of(9.08), Seconds.of(2.5));
    // Add more entries as needed
    INSTANCE = new FeedingTimeTable(timeTable);
  }

  public static FeedingTimeTable getInstance() {
    return INSTANCE;
  }

  private FeedingTimeTable(Map<Distance, Time> timeTable) {
    super(timeTable);
  }
}
