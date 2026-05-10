// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * CIM Motor subsystem using TalonSRX on CAN ID 30. Controls a CIM motor at 10% speed moving back
 * and forth.
 */
public class CIMotorSubsystem extends SubsystemBase {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(30);

  private boolean movingForward = true;
  private static final double SPEED = 0.10; // 10% speed

  public CIMotorSubsystem() {
    motor.configFactoryDefault();
  }

  /** Move the motor at 10% speed in the current direction. */
  public void move() {
    double output = movingForward ? SPEED : -SPEED;
    motor.set(output);
  }

  /** Stop the motor. */
  public void stop() {
    motor.set(0);
  }

  /** Toggle the direction of movement. */
  public void toggleDirection() {
    movingForward = !movingForward;
  }

  /**
   * Get the current direction.
   *
   * @return true if moving forward, false if moving backward
   */
  public boolean isMovingForward() {
    return movingForward;
  }

  /**
   * Set the direction explicitly.
   *
   * @param forward true for forward, false for backward
   */
  public void setDirection(boolean forward) {
    movingForward = forward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
