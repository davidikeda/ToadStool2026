// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CIMotorSubsystem;

public class MoveCIMotorCommand extends Command {
  private final CIMotorSubsystem cimMotor;
  private static final double TOGGLE_INTERVAL = 2.0; // seconds
  private double lastToggleTime = 0;

  public MoveCIMotorCommand(CIMotorSubsystem cimMotor) {
    this.cimMotor = cimMotor;
    addRequirements(cimMotor);
  }

  @Override
  public void initialize() {
    lastToggleTime = 0;
    cimMotor.setDirection(true);
  }

  @Override
  public void execute() {
    // Move the motor
    cimMotor.move();
  }

  @Override
  public void end(boolean interrupted) {
    cimMotor.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs indefinitely until interrupted
  }
}
