package frc.robot.autos;

import java.util.function.Function;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public enum Autos {

    NO_AUTO(AutoFactory::noAuto),

    // preload
    PRELOAD_ONLY_LEFT(AutoFactory::preloadOnlyLeft),
    PRELOAD_ONLY_RIGHT(AutoFactory::preloadOnlyRight),
    PRELOAD_ONLY_CENTER(AutoFactory::preloadOnlyCenter),

    // left
    LEFT_SINGLE_SWEEP(AutoFactory::leftSingleTrenchSweep),
    LEFT_DOUBLE_SWEEP(AutoFactory::leftDoubleSweep),

    // right
    RIGHT_SINGLE_SWEEP(AutoFactory::rightSingleTrenchSweep),
    RIGHT_DOUBLE_SWEEP(AutoFactory::rightDoubleSweep),

    // center
    CENTER_DEPOT_OUTPOST(AutoFactory::centerDepotOutpost),
    CENTER_OUTPOST_DEPOT(AutoFactory::centerDepotOutpost);


  public final Function<AutoFactory, Pair<Pose2d, Command>> command;

  private Autos(Function<AutoFactory, Pair<Pose2d, Command>> specificCommand) {
    command = specificCommand;
  }

  public Pose2d getStartPose(AutoFactory allianceFactory) {
    return command.apply(allianceFactory).getFirst();
  }

  public Command getAutoRoutine(AutoFactory allianceFactory) {
    return command.apply(allianceFactory).getSecond();
  }

  public Pair<Pose2d, Command> getStartPoseAndRoutine(AutoFactory allianceFactory) {
    return command.apply(allianceFactory);
  }
}
