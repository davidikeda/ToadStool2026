// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.config.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CommandXboxController controller = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /* Default Commands */
    // Drive
    drivetrain.setDefaultCommand(
      new DrivetrainCommand(
        drivetrain,
        DrivetrainCommand.Position.TELEOP,
        controller::getLeftX,
        controller::getLeftY,
        controller::getRightX));

    /* Controls */
    // Thers nothing here yet

}

public CommandSwerveDrivetrain getDrivetrain() {
  return drivetrain;
}
}
