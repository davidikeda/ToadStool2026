package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ToadSwerveModules extends SubsystemBase {
  public static class AS5600Encoder {
    private final I2C i2c;
    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REG = 0x0E;

    public AS5600Encoder(I2C.Port port, int address) {
      this.i2c = new I2C(port, address);
    }

    public double getAngleDegrees() {
      byte[] rawData = new byte[2];
      i2c.read(ANGLE_REG, 2, rawData);

      int raw12bit = ((rawData[0] & 0xFF) << 4) | ((rawData[1] & 0xFF) >> 4);

      return (raw12bit / 4095.0) * 360.0;
    }
  }

  public class SwerveModule {
    private final WPI_VictorSPX driveMotor;
    private final WPI_VictorSPX steerMotor;
    private final AS5600Encoder turnEncoder;
    private final PIDController turnPID;

    private static final double MAX_DRIVE_SPEED_MPS = 5.0;

    public SwerveModule(int driveMotorID, int steerMotorID, I2C.Port i2cPort, int encoderAddress) {
      driveMotor = new WPI_VictorSPX(driveMotorID);
      steerMotor = new WPI_VictorSPX(steerMotorID);
      turnEncoder = new AS5600Encoder(i2cPort, encoderAddress);

      turnPID = new PIDController(1.0, 0.0, 0.0);
      turnPID.enableContinuousInput(-180.0, 180.0);
    }

    public void setDesiredState(double speedMetersPerSecond, double angleDegrees) {
      double currentAngle = turnEncoder.getAngleDegrees();
      double delta = Math.IEEEremainder(angleDegrees - currentAngle, 360.0);
      if (Math.abs(delta) > 90.0) {
        delta = Math.IEEEremainder(delta + 180.0, 360.0);
        speedMetersPerSecond = -speedMetersPerSecond;
      }

      double targetAngle = currentAngle + delta;

      double turnOutput = turnPID.calculate(currentAngle, targetAngle);
      turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
      steerMotor.set(turnOutput);

      double drivePercent = MathUtil.clamp(speedMetersPerSecond / MAX_DRIVE_SPEED_MPS, -1.0, 1.0);
      driveMotor.set(drivePercent);
    }

    public SwerveModuleState getState() {
      double drivePercent = driveMotor.get();
      double speedMetersPerSecond = drivePercent * MAX_DRIVE_SPEED_MPS;
      Rotation2d angle = Rotation2d.fromDegrees(turnEncoder.getAngleDegrees());
      return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public void stop() {
      driveMotor.stopMotor();
      steerMotor.stopMotor();
    }
  }

  /* private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      // Use config to set up auto builder parameters
      AutoBuilder.configure(
          () -> getState().Pose,
          this::resetPose,
          () -> getState().Speeds,
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(7, 0, 0)),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to configure AutoBuilder: " + e.getMessage(), e.getStackTrace());
    }
  }
  */

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private static final double MAX_DRIVE_SPEED_MPS = 5.0;

  public ToadSwerveModules(
      double wheelbaseInches,
      double trackwidthInches,
      I2C.Port i2cPort,
      int flDriveId,
      int flSteerId,
      int flEncoderAddr,
      int frDriveId,
      int frSteerId,
      int frEncoderAddr,
      int blDriveId,
      int blSteerId,
      int blEncoderAddr,
      int brDriveId,
      int brSteerId,
      int brEncoderAddr) {

    frontLeft = new SwerveModule(flDriveId, flSteerId, i2cPort, flEncoderAddr);
    frontRight = new SwerveModule(frDriveId, frSteerId, i2cPort, frEncoderAddr);
    backLeft = new SwerveModule(blDriveId, blSteerId, i2cPort, blEncoderAddr);
    backRight = new SwerveModule(brDriveId, brSteerId, i2cPort, brEncoderAddr);

    double halfWheelbase = inchesToMeters(wheelbaseInches) / 2.0;
    double halfTrack = inchesToMeters(trackwidthInches) / 2.0;

    Translation2d flPos = new Translation2d(+halfWheelbase, +halfTrack);
    Translation2d frPos = new Translation2d(+halfWheelbase, -halfTrack);
    Translation2d blPos = new Translation2d(-halfWheelbase, +halfTrack);
    Translation2d brPos = new Translation2d(-halfWheelbase, -halfTrack);

    kinematics = new SwerveDriveKinematics(flPos, frPos, blPos, brPos);
    odometry =
        new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(0.0),
            new edu.wpi.first.math.kinematics.SwerveModulePosition[] {
              new edu.wpi.first.math.kinematics.SwerveModulePosition(
                  0.0, Rotation2d.fromDegrees(0.0)),
              new edu.wpi.first.math.kinematics.SwerveModulePosition(
                  0.0, Rotation2d.fromDegrees(0.0)),
              new edu.wpi.first.math.kinematics.SwerveModulePosition(
                  0.0, Rotation2d.fromDegrees(0.0)),
              new edu.wpi.first.math.kinematics.SwerveModulePosition(
                  0.0, Rotation2d.fromDegrees(0.0))
            });
  }

  public ToadSwerveModules() {
    this(15.5, 15.5, I2C.Port.kOnboard, 1, 2, 0x36, 3, 4, 0x36, 11, 12, 0x36, 13, 14, 0x36);
  }

  /** drive helper: x forward (m/s), y left (m/s), rot CCW (rad/s). */
  public void drive(
      double xMetersPerSec, double yMetersPerSec, double rotRadPerSec, boolean fieldRelative) {
    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSec, yMetersPerSec, rotRadPerSec, Rotation2d.fromDegrees(0.0))
            : new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotRadPerSec);

    var states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_SPEED_MPS);

    frontLeft.setDesiredState(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
    frontRight.setDesiredState(states[1].speedMetersPerSecond, states[1].angle.getDegrees());
    backLeft.setDesiredState(states[2].speedMetersPerSecond, states[2].angle.getDegrees());
    backRight.setDesiredState(states[3].speedMetersPerSecond, states[3].angle.getDegrees());
  }

  public void stopAll() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  private static double inchesToMeters(double in) {
    return in * 0.0254;
  }

  /*
  public void setPose(Pose2d pose) {
    this.resetPose(pose);
  }
  */

  public Object getNavX() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getNavX'");
  }
}
