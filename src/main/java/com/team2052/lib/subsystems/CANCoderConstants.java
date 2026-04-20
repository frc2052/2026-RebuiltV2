package com.team2052.lib.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Frequency;
import lombok.Getter;
import lombok.Setter;

public class CANCoderConstants {
  public @Getter @Setter Pair<Integer, CANBus> id;
  public @Getter @Setter CANcoderConfiguration config = new CANcoderConfiguration();
  public @Getter @Setter Frequency statusSignalUpdateFrequency = Hertz.of(50);
}
