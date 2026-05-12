// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ToadSwerveModules;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Pose2d robotPose = new Pose2d();
  private final RobotContainer m_robotContainer;
  private ToadSwerveModules swerveModules = new ToadSwerveModules();
  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("robotPose", Pose2d.struct).publish();
  Field2d field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    swerveModules = m_robotContainer.getSwerveModules();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
    // Temp fix until I make a function to get the yaw from the navx
    // SmartDashboard.putNumber("Gyro YAW", swerveModules.getNavX().getYaw());
    // robotPose = swerveModules.getPose();
    publisher.set(robotPose);
    field.setRobotPose(robotPose);
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
