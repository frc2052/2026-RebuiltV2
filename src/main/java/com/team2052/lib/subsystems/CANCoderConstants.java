package com.team2052.lib.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Frequency;

public class CANCoderConstants {
  public Pair<Integer, CANBus> id;
  public CANcoderConfiguration config = new CANcoderConfiguration();
  public Frequency statusSignalUpdateFrequency = Hertz.of(50);
}
