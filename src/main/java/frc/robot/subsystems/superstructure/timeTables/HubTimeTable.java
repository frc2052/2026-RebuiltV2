package frc.robot.subsystems.superstructure.timeTables;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class HubTimeTable extends TimeTableBase {

    private static HubTimeTable INSTANCE;

    static {
        Map<Distance, Time> timeTable = new HashMap<>();
        // Initialize the time table with distance and corresponding time pairs
        timeTable.put(Meters.of(10), Seconds.of(1.5));
        timeTable.put(Meters.of(20), Seconds.of(2.0));
        // Add more entries as needed
        INSTANCE = new HubTimeTable(timeTable);
    }

    public static HubTimeTable getInstance() {
        return INSTANCE;
    }

    private HubTimeTable(Map<Distance, Time> timeTable) {
        super(timeTable);
    }
    
}
