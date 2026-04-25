package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

public enum Autos {
  NO_AUTO(AutoFactory::noAuto),

  // preload
  PRELOAD_ONLY_LEFT(AutoFactory::preloadOnlyLeft),
  PRELOAD_ONLY_RIGHT(AutoFactory::preloadOnlyRight),
  PRELOAD_ONLY_CENTER(AutoFactory::preloadOnlyCenter),

  // left
  LEFT_SINGLE_SWEEP(AutoFactory::leftSingleTrenchSweep), 
  LEFT_DOUBLE_SWEEP(AutoFactory::leftDoubleSweep), 
  DEPOT(AutoFactory::Depot),

  // right
  RIGHT_SINGLE_SWEEP(AutoFactory::rightSingleTrenchSweep), 
  RIGHT_DOUBLE_SWEEP(AutoFactory::rightDoubleSweep), 
  RIGHT_FULL_SWEEP(AutoFactory::rightFullSweep),

  // // other -> requires a unique method of usage
  // RIGHT_PRELOAD_OUTPOST(AutoFactory::rightPreloadOutpost),
  // LEFT_DELAY_BUMP_SWIPE(AutoFactory::leftDelayBumpSwipe),
  // RIGHT_DELAY_BUMP_SWIPE_OUTPOST(AutoFactory::rightDelayBumpSwipeOutpost),

  // // delayed trench swipe
  LEFT_DELAY_TRENCH_SWIPE(AutoFactory::leftDelayTrenchSwipe),
  RIGHT_DELAY_TRENCH_SWIPE(AutoFactory::rightDelayTrenchSwipe);


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
