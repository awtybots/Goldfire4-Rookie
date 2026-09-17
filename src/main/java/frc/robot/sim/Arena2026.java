package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class Arena2026 extends SimulatedArena {

  public Arena2026() {
    super(new Map2026());
  }

  @Override
  public void placeGamePiecesOnField() {}

  private static final class Map2026 extends FieldMap {

    private Map2026() {
      double length = SimRobot.SimConstants.FIELD_LENGTH_M;
      double width = SimRobot.SimConstants.FIELD_WIDTH_M;

      addBorderLine(new Translation2d(0, 0), new Translation2d(length, 0));
      addBorderLine(new Translation2d(length, 0), new Translation2d(length, width));
      addBorderLine(new Translation2d(length, width), new Translation2d(0, width));
      addBorderLine(new Translation2d(0, width), new Translation2d(0, 0));

      for (double[] box : SimRobot.OBSTACLES) {
        addRectangularObstacle(box[1] - box[0], box[3] - box[2],
            new Pose2d((box[0] + box[1]) / 2.0, (box[2] + box[3]) / 2.0, Rotation2d.kZero));
      }
    }
  }
}
