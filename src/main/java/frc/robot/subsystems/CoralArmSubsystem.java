// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Common.ArmController;
import frc.Common.EncoderVelocityTracker;
import frc.Common.ArmController.AngleControlState;
import frc.Common.ArmController.AngleUnit;
import frc.Common.ConstraintClasses.RangeConstraint;
import frc.robot.Constants.*;

public class CoralArmSubsystem extends SubsystemBase {
  /** Creates a new CoralArmSubsystem. */

  private SparkMax angleMotor = new SparkMax(CoralArmConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralArmConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig angleConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(CoralArmConstants.ABSOLUTE_ENCODER_PORT);
  private EncoderVelocityTracker encoderVelocity = new EncoderVelocityTracker(this::getRawAngle);

  private ArmController arm = new ArmController(
    angleMotor::set,
    this::getRawAngle,
    this::getAngularVelocity,

    CoralArmConstants.ABSOLUTE_ENCODER_OFFSET_DEGREES,
    CoralArmConstants.ANGLE_SETPOINT_TOLERANCE_DEGREES,
    new RangeConstraint(-90, 90),

    CoralArmConstants.ANGLE_PID_P,
    CoralArmConstants.ANGLE_PID_I,
    CoralArmConstants.ANGLE_PID_D,

    CoralArmConstants.FF_KS,
    CoralArmConstants.FF_KG,
    CoralArmConstants.FF_KV,

    CoralArmConstants.MAX_ANGULAR_VELOCITY_DEG_SEC,
    CoralArmConstants.MAX_PROFILED_ANGULAR_ACCELERATION_DEG_SEC_SEC,

    "Coral Arm",
    AngleUnit.DEGREES
  );

  private Alert hardwareFaultAlert = new Alert("Coral arm control has been disabled due to a hardware fault", AlertType.kError);

  public CoralArmSubsystem() {
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(CoralArmConstants.INTAKE_MOTOR_SMART_CURRENT_LIMIT);
    intakeConfig.inverted(CoralArmConstants.INTAKE_MOTOR_INVERTED);
    angleConfig.idleMode(IdleMode.kBrake);
    angleConfig.smartCurrentLimit(CoralArmConstants.ANGLE_MOTOR_SMART_CURRENT_LIMIT);
    angleConfig.inverted(CoralArmConstants.ANGLE_MOTOR_INVERTED);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    absoluteEncoder.setInverted(CoralArmConstants.ABSOLUTE_ENCODER_INVERTED);

    if (!absoluteEncoder.isConnected()) {
      arm.disable();
      hardwareFaultAlert.setText(hardwareFaultAlert.getText() + ": Absolute encoder fault");
      hardwareFaultAlert.set(true);
    }
  }

  public double getRawAngle() {
    return (absoluteEncoder.get() * 360.0);
  }

  public double getAngularVelocity() {
    return encoderVelocity.getVelocity();
  }

  public double getAngle() {
    return arm.getAngle();
  }

  public void setPosition(double angleDegrees) {
    arm.setAngle(angleDegrees);
  }

  public void setProfiled(double angleDegrees) {
    arm.setProfiled(angleDegrees);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  //Enables angle control
  public void enable() {
    if (absoluteEncoder.isConnected()) {
      arm.enable();
      hardwareFaultAlert.set(false);
    }
    else {
      arm.disable();
      hardwareFaultAlert.set(true);
    }
  }

  //Disables angle control
  public void disable() {
    arm.disable();
  }

  public boolean isEnabled() {
    return arm.getState() != AngleControlState.DISABLED;
  }

  public boolean atSetpoint() {
    return arm.atSetpoint();
  }

  @Override
  public void periodic() {
    arm.execute();
    encoderVelocity.update();
  }
}
