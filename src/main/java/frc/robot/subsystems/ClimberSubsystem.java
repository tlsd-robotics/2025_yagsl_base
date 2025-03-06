// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private SparkMax motor = new SparkMax(ClimberConstants.MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private DutyCycleEncoder encoder = new DutyCycleEncoder(ClimberConstants.ABSOLUTE_ENCODER_PORT);

  private double speed;

  public ClimberSubsystem() {
    config.inverted(ClimberConstants.MOTOR_INVERTED);
    config.smartCurrentLimit(ClimberConstants.SMART_CURRENT_LIMIT_AMPS);
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setInverted(ClimberConstants.ENCODER_IVNERTED);
  }

  public double getAngle() {
    return MathUtil.inputModulus(encoder.get() * 360 - ClimberConstants.ABSOLUTE_ENCODER_OFFSET, -20, 340);
  }

  public void set(double speed) {
    this.speed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double angle = getAngle();

    SmartDashboard.putNumber("Climber Angle: ", angle);

    if (angle >= ClimberConstants.ANGLE_RANGE_DEGREES.getUpperConstraint() && speed <= 0) {
      motor.set(speed);
    }
    else if (angle <= ClimberConstants.ANGLE_RANGE_DEGREES.getLowerConstraint() && speed >= 0) {
      motor.set(speed);
    }
    else if (ClimberConstants.ANGLE_RANGE_DEGREES.inRange(angle)) {
      motor.set(speed);
    }
    else {
      motor.set(0);
    }
  }
}
