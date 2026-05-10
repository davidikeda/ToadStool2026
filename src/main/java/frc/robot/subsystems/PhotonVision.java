package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.VisionConfig;
import org.photonvision.PhotonCamera;

public class PhotonVision extends SubsystemBase {
  public static final PhotonCamera FrontCamera = new PhotonCamera(VisionConfig.FRONT_CAMERA);
  public static final PhotonCamera BackCamera = new PhotonCamera(VisionConfig.BACK_CAMERA);
}
