package frc.robot.subsystems.superstructure.timeTables;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class TimeTableBase {
    
    private final Map<Distance, Time> timeTable;
    private final InterpolatingDoubleTreeMap timeMap;

    public TimeTableBase(Map<Distance, Time> timeTable) {
        this.timeTable = timeTable;
        this.timeMap = new InterpolatingDoubleTreeMap();
        for (Map.Entry<Distance, Time> entry : timeTable.entrySet()) {
            Distance distance = entry.getKey();
            Time time = entry.getValue();
            timeMap.put(distance.in(Meters), time.in(Seconds));
        }
    }

    public Map<Distance, Time> getTimeTable() {
        return timeTable;
    }

    public Time getTime(Distance distance) {
        double distanceMeters = distance.in(Meters);
        double timeSeconds = timeMap.get(distanceMeters);
        return Seconds.of(timeSeconds);
    }
}
