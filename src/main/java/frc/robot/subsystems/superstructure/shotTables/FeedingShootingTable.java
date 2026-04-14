package frc.robot.subsystems.superstructure.shotTables;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class FeedingShootingTable extends ShootingTableBase {

    private static FeedingShootingTable INSTANCE;
    
    static {
        Map<Distance, Pair<AngularVelocity, Angle>> shootingTable = new HashMap<>();
        // Initialize the shooting table with distance, shooter velocity, and hood angle pairs
        shootingTable.put(Meters.of(10), new Pair<>(RotationsPerSecond.of(40), Degrees.of(45)));
        shootingTable.put(Meters.of(20), new Pair<>(RotationsPerSecond.of(40), Degrees.of(50)));
        // Add more entries as needed
        INSTANCE = new FeedingShootingTable(shootingTable);
    }
    
    public static FeedingShootingTable getInstance() {
        return INSTANCE;
    }

    private FeedingShootingTable(Map<Distance, Pair<AngularVelocity, Angle>> shootingTable) {
        super(shootingTable);
    }
    
}
