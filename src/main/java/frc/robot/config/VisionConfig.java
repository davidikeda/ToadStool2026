package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionConfig {
  public static PhotonPipelineResult result = new PhotonPipelineResult();
  public static final String FRONT_CAMERA = "front";
  // grayson adjust these
  public static final Matrix<N3, N1> FRONT_VISION_STDDEVS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3.0));
  public static final String BACK_CAMERA = "back";
  // grayson adjust these
  public static final Matrix<N3, N1> BACK_VISION_STDDEVS =
      VecBuilder.fill(0.10, 0.10, Units.degreesToRadians(5.0));
  // Depending how bad you wnt vision to work you can make the april tag layout for the off season
  // events but I aint doin that shit again
  public static AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static final PhotonPoseEstimator.PoseStrategy STRATEGY =
      PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
  // grayson adjust these
  public static final Transform3d FRONT_CAMERA_POS = new Transform3d();
  public static final Transform3d BACK_CAMERA_POS = new Transform3d();
}
