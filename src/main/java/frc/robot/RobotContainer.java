// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.config.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final ToadSwerveModules swerveModules = new ToadSwerveModules();
  private final CIMotorSubsystem cimMotor = new CIMotorSubsystem();
  private ToadSwerveModules drivestate = swerveModules.getState();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* Default Commands */
    // Drive
    swerveModules.setDefaultCommand(new SwerveCommand(swerveModules, driverController));
    /* Button Bindings */

  }

  public ToadSwerveModules getSwerveModules() {
    return swerveModules;
  }

  public CIMotorSubsystem getCIMotor() {
    return cimMotor;
  }
}
