package frc.robot.subsystems.superstructure.shotTables;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.*;
import java.util.HashMap;
import java.util.Map;

public class HubShootingTable extends ShootingTableBase {

  private static HubShootingTable INSTANCE;

  // use the superstructure one if you want to affect feeding as well
  private static final double squishMod = 0; // 0.75 + 1.25

  static {
    Map<Distance, Pair<AngularVelocity, Angle>> shootingTable = new HashMap<>();
    // Initialize the shooting table with distance, shooter velocity, and hood angle pairs
    shootingTable.put(
        Meters.of(1.6), new Pair<>(RotationsPerSecond.of(32.5 + squishMod), Degrees.of(7.9)));
    shootingTable.put(
        Meters.of(2), new Pair<>(RotationsPerSecond.of(33.5 + squishMod), Degrees.of(9.8)));
    shootingTable.put(
        Meters.of(2.5), new Pair<>(RotationsPerSecond.of(35.5 + squishMod), Degrees.of(12.8)));
    shootingTable.put(
        Meters.of(3), new Pair<>(RotationsPerSecond.of(38 + squishMod), Degrees.of(14.3)));
    shootingTable.put(
        Meters.of(3.5), new Pair<>(RotationsPerSecond.of(39.5 + squishMod), Degrees.of(17.8)));
    shootingTable.put(
        Meters.of(4), new Pair<>(RotationsPerSecond.of(41.5 + squishMod), Degrees.of(19.8)));
    shootingTable.put(
        Meters.of(4.5), new Pair<>(RotationsPerSecond.of(44 + squishMod), Degrees.of(25.3)));

    // Add more entries as needed
    INSTANCE = new HubShootingTable(shootingTable);
  }

  public static HubShootingTable getInstance() {
    return INSTANCE;
  }

  private HubShootingTable(Map<Distance, Pair<AngularVelocity, Angle>> shootingTable) {
    super(shootingTable);
  }
}
