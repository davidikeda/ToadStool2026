// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.config.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CIMotorSubsystem cimMotor = new CIMotorSubsystem();

  private final CommandJoystick driverController = new CommandJoystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* Default Commands */
    // Drive
    drivetrain.setDefaultCommand(
        new DrivetrainCommand(
            drivetrain,
            DrivetrainCommand.Position.TELEOP,
            driverController::getX,
            driverController::getY,
            driverController::getTwist));

    // CIM Motor - move back and forth at 10% speed when button 1 is held
    cimMotor.setDefaultCommand(new MoveCIMotorCommand(cimMotor));

    // Button 1 triggers CIM motor command
    driverController.button(1).whileTrue(new MoveCIMotorCommand(cimMotor));
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public CIMotorSubsystem getCIMotor() {
    return cimMotor;
  }
}
