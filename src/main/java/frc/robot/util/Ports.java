package frc.robot.util;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Pair;

public class Ports {
  /*
   * CAN:
   * A Pair holds both CAN ID and CAN Bus for devices
   * When creating a device, use .getFirst() for ID and .getSecond() for Bus
   */
  public static final CANBus MAIN_BUS = new CANBus("Krawlivore");
  public static final CANBus RIO_BUS = CANBus.roboRIO();

  public static final Pair<Integer, CANBus> FLOOR_LEFT_MOTOR = Pair.of(6, MAIN_BUS);
  public static final Pair<Integer, CANBus> FLOOR_RIGHT_MOTOR = Pair.of(7, MAIN_BUS);

  public static final Pair<Integer, CANBus> SHOOTER_TOP_LEFT = Pair.of(2, MAIN_BUS);
  public static final Pair<Integer, CANBus> SHOOTER_BOTTOM_LEFT = Pair.of(3, MAIN_BUS);
  public static final Pair<Integer, CANBus> SHOOTER_TOP_RIGHT = Pair.of(4, MAIN_BUS);
  public static final Pair<Integer, CANBus> SHOOTER_BOTTOM_RIGHT = Pair.of(5, MAIN_BUS);

  public static final Pair<Integer, CANBus> STAGER_TOP_MOTOR = Pair.of(6, MAIN_BUS);
  public static final Pair<Integer, CANBus> STAGER_BOTTOM_MOTOR = Pair.of(7, MAIN_BUS);

  public static final Pair<Integer, CANBus> HOOD_MOTOR = Pair.of(8, MAIN_BUS);
  public static final Pair<Integer, CANBus> HOOD_ENCODER = Pair.of(9, MAIN_BUS);

}
