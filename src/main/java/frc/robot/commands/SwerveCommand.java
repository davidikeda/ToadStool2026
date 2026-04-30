package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ToadSwerveModules;

public class SwerveCommand extends Command {
  private final ToadSwerveModules swerve;
  private final CommandXboxController controller;

  private static final double MAX_LINEAR_SPEED_MPS = 3.0; // max speed in mps
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI; // max angle
  private static final double DEADBAND = 0.05;

  public SwerveCommand(ToadSwerveModules swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double xInput = MathUtil.applyDeadband(controller.getLeftY(), DEADBAND);
    double yInput = MathUtil.applyDeadband(controller.getLeftX(), DEADBAND);
    double rotInput = MathUtil.applyDeadband(controller.getRightX(), DEADBAND);

    double xMetersPerSec = xInput * MAX_LINEAR_SPEED_MPS;
    double yMetersPerSec = yInput * MAX_LINEAR_SPEED_MPS;
    double rotRadPerSec = rotInput * MAX_ANGULAR_SPEED_RAD_PER_SEC;
    swerve.drive(
        xMetersPerSec, yMetersPerSec, rotRadPerSec, false); // set false unless playing on field
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted
  }
}
