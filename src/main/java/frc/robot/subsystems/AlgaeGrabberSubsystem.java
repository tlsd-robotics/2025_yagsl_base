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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class AlgaeGrabberSubsystem extends SubsystemBase {
  /** Creates a new AlgaeGrabberSubsystem. */

  enum AngleControlState {
    POSITION_CONTROL("Position Control"),
    DISABLED("Disabled"),
    FAULT("Hardware Fault");

    String displayName;

    AngleControlState(String displayName) {
      this.displayName = displayName;
    };
  }



  private SparkMax angleMotor = new SparkMax(AlgaeGrabberConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(AlgaeGrabberConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig angleConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(AlgaeGrabberConstants.ABSOLUTE_ENCODER_PORT);

  private PIDController anglePID = new PIDController(
    AlgaeGrabberConstants.ANGLE_PID_P, 
    AlgaeGrabberConstants.ANGLE_PID_I,
    AlgaeGrabberConstants.ANGLE_PID_D
  );

  private ArmFeedforward angleFF = new ArmFeedforward(
    AlgaeGrabberConstants.ANGLE_FF_KS,
    AlgaeGrabberConstants.ANGLE_FF_KG,
    AlgaeGrabberConstants.ANGLE_FF_KV
  );

  private AngleControlState currentAngleControlState = AngleControlState.DISABLED;
  private double setpoint = 0.0;

  private Alert hardwareFaultAlert = new Alert("Algae arm control has been disabled due to a hardware fault", AlertType.kError);

  public AlgaeGrabberSubsystem() {
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(AlgaeGrabberConstants.INTAKE_MOTOR_SMART_CURRENT_LIMIT);
    intakeConfig.inverted(AlgaeGrabberConstants.INTAKE_MOTOR_INVERTED);
    angleConfig.idleMode(IdleMode.kBrake);
    angleConfig.smartCurrentLimit(AlgaeGrabberConstants.ANGLE_MOTOR_SMART_CURRENT_LIMIT);
    angleConfig.inverted(AlgaeGrabberConstants.ANGLE_MOTOR_INVERTED);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    absoluteEncoder.setInverted(false);

    if (!absoluteEncoder.isConnected()) {
      currentAngleControlState = AngleControlState.FAULT;
      hardwareFaultAlert.setText(hardwareFaultAlert.getText() + ": Absolute encoder fault");
      hardwareFaultAlert.set(true);
    }
  }


  //Returns the current angle of the arm in degrees
  public double getAngle() {

    if (!absoluteEncoder.isConnected()) {
      hardwareFaultAlert.setText(hardwareFaultAlert.getText() + ": Absolute encoder fault");
      hardwareFaultAlert.set(true);

      currentAngleControlState = AngleControlState.FAULT;
      return 0;
    }

    return (absoluteEncoder.get() * 360.0) - AlgaeGrabberConstants.ABSOLUTE_ENCODER_OFFSET_DEGREES;
  }

  public void set(double angleDegrees) {
    setpoint = AlgaeGrabberConstants.ANGLE_RANGE_DEGREES.clamp(angleDegrees);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  //Enables angle control
  public void enable() {
    if (absoluteEncoder.isConnected()) {
      setpoint = getAngle();
      currentAngleControlState = AngleControlState.POSITION_CONTROL;
      hardwareFaultAlert.set(false);
    }
    else {
      currentAngleControlState = AngleControlState.FAULT;
    }
  }

  //Disables angle control
  public void disable() {
    if (currentAngleControlState != AngleControlState.FAULT) {
      currentAngleControlState = AngleControlState.DISABLED;
    }
  }

  public AngleControlState getAngleControlState() {
    return currentAngleControlState;
  }

  public boolean isEnabled() {
    return currentAngleControlState == AngleControlState.POSITION_CONTROL;
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(setpoint, getAngle(), AlgaeGrabberConstants.ANGLE_SETPOINT_TOLERANCE_DEGREES);
  }



  @Override
  public void periodic() {
    //Output data to smart dashboard:
    SmartDashboard.putString("Algae Arm State: ", getAngleControlState().displayName);
    SmartDashboard.putNumber("Algae Arm Current Angle (Degrees): ", getAngle());
    SmartDashboard.putNumber("Algae Arm Angle Setpoint (Degrees): ", setpoint);
    SmartDashboard.putBoolean("Algae Arm at Setpoint: ", atSetpoint());
    SmartDashboard.putNumber("Algae Arm Angle Motor Output Power: ", angleMotor.getAppliedOutput());

    //Angle control:
    switch (currentAngleControlState) {
      case POSITION_CONTROL:
        SmartDashboard.putNumber("pid calculation: ", anglePID.calculate(getAngle(), setpoint));
        angleMotor.set(anglePID.calculate(getAngle(), setpoint) + angleFF.calculate(Units.degreesToRadians(setpoint), 0));
        break;

      case DISABLED:
      case FAULT:
        angleMotor.set(0);
        break;
    }
  }
}
