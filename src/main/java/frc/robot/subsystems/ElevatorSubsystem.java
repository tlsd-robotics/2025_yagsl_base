// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

//TODO: Fix stall detection by averaging speed over time. It is currently detecting stops from imperfections in the racks.

public class ElevatorSubsystem extends SubsystemBase {

  enum ElevatorState {
    DISABLED("Disabled"),
    POSITION_CONTROL("Position Control"),
    PROFILED_CONTROL("Profiled Control"),
    HOMING("Auto-Homing");

    String displayName;

    ElevatorState(String displayName) {
      this.displayName = displayName;
    };
  }

  private SparkMax elevator = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder elevatorRelativeEncoder = elevator.getEncoder();
  private SparkMaxConfig config = new SparkMaxConfig();
  private double setPoint = ElevatorConstants.SETPOINT_HOME;

  private PIDController elevatorPID = new PIDController(
    ElevatorConstants.PID_P,
    ElevatorConstants.PID_I,
    ElevatorConstants.PID_D
  );

  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 0,0,0);

  private TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.ELEVATOR_MAX_SPEED_MS, ElevatorConstants.ELEVATOR_ALLOWED_ACCEL_MSS));
  private TrapezoidProfile.State currentInitialState = null;
  private TrapezoidProfile.State currentTargetState  = null;

  private Timer motionProfileTimer = new Timer();
  private Timer homingTimer        = new Timer();

  private ElevatorState currentState = ElevatorState.DISABLED;

  private Alert noOpSetAlert = new Alert("Attempted to change elevator setpoint with control disabled", AlertType.kInfo);
  private Alert stallDetectedAlert = new Alert("Stall Detected. Elevator Control Disabled. AUTOHOME IMMEDIATLEY UPON REENABLE!", AlertType.kError);

  //PUBLIC METHODS
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.inverted(ElevatorConstants.ELEVATOR_MOTOR_INVERTED);
    config.encoder.positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
    config.encoder.velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR);
    elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRelativeEncoder.setPosition(0); //Asume elevator is starting from the bottom
  }

  public double getPosition() {
    return elevatorRelativeEncoder.getPosition();
  }

  public double getVelocity() {
    return elevatorRelativeEncoder.getVelocity();
  }

  public void setProfiled(double setPoint) {

    if (!isEnabled()) {
      noOpSetAlert.set(true);
      return;
    }

    this.setPoint = setPoint; //set the non-profiled setpoint to allow atSetpoint() to work correctly.

    //Creates states for the trapezoid profile with the current state and a state at the target position with zero velocity.
    currentInitialState = new TrapezoidProfile.State(getPosition(), getVelocity());
    currentTargetState =  new TrapezoidProfile.State(MathUtil.clamp(setPoint, 0, ElevatorConstants.ELEVATOR_MAX_HEIGHT_M), 0);
    motionProfileTimer.restart();
    currentState = ElevatorState.PROFILED_CONTROL;
  }

  public void setPosition(double setPoint) {

    if (!isEnabled()) {
      noOpSetAlert.set(true);
      return;
    }

    currentState = ElevatorState.POSITION_CONTROL; //set elevator to postion control mode
    this.setPoint = setPoint;
  }

  public void autoHome() {

    currentState = ElevatorState.HOMING;
    homingTimer.restart();

  }
  
  public void enable() {
    setPoint = getPosition(); //change setpoint to current position to prevent erratic movement
    currentState = ElevatorState.POSITION_CONTROL;
  }

  public void disable() {
    currentState = ElevatorState.DISABLED;
  }

  public boolean isEnabled() {
    return currentState == ElevatorState.POSITION_CONTROL || currentState == ElevatorState.PROFILED_CONTROL;
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(setPoint, getPosition(), ElevatorConstants.SETPOINT_TOLERANCE);
  }

  public boolean atVelocity(double velocity) {
    return MathUtil.isNear(velocity, getVelocity(), ElevatorConstants.VELOCITY_TOLERANCE_MS);
  }

  @Override
  public void periodic() {

    //Output data to smartdashboard
    SmartDashboard.putNumber("Elevator Position (Inches): ", Units.metersToInches(getPosition()));
    SmartDashboard.putString("Elevator Mode: ", currentState.displayName);
    SmartDashboard.putNumber("Elevator Setpoint (Inches): ", Units.metersToInches(setPoint));
    SmartDashboard.putNumber("Elevator Motor Output Percent", elevator.getAppliedOutput());
    SmartDashboard.putBoolean("Elevator at Setpoint: ", atSetpoint());
    SmartDashboard.putNumber("Elevator velocity (M/S): ", getVelocity());

    //Primary control handling
    switch (currentState) {
      case DISABLED:

        elevator.set(0);
        break;

      case POSITION_CONTROL:

        if ((setPoint == 0) && atSetpoint()) { //Turn off motors if resting on bottom
         elevator.set(0);
        }
        else {
          elevator.set(elevatorPID.calculate(getPosition(), setPoint));
        }

        break;

      case PROFILED_CONTROL:

        TrapezoidProfile.State target = motionProfile.calculate(motionProfileTimer.get(), currentInitialState, currentTargetState);
        elevator.set(elevatorPID.calculate(getPosition(), MathUtil.clamp(target.position, 0, ElevatorConstants.ELEVATOR_MAX_HEIGHT_M)) + elevatorFF.calculate(target.velocity));

        if (target.position == setPoint) { //Once profiled motion velocity reaches zero, switch to normal control to hold position.
          currentState = ElevatorState.POSITION_CONTROL;
        }

        break;
        
      case HOMING:

        elevator.set(-Math.abs(ElevatorConstants.AUTOHOME_OUTPUT_PERCENT)); //Move the elevator down at the given rate
        
        //if movement has stopped and time has elapsed assume elevator has hit the bottom
        if (homingTimer.hasElapsed(ElevatorConstants.AUTOHOME_WAIT_TIME_SEC) && atVelocity(0)) {
          elevatorRelativeEncoder.setPosition(0);
          setPoint = 0;
          currentState = ElevatorState.POSITION_CONTROL;
          elevator.set(0);
        }
    }

    //Stall Detection:
    //If the elevator is not moving, is not at its setpoint, and the motor is using a current over STALL_THRESHOLD_AMPS, 
    //assume the elevator is stuck and possibly zeroed improperly. Disable elevator control.
    if (currentState != ElevatorState.HOMING && atVelocity(0) && 
        !atSetpoint() && elevator.getOutputCurrent() >= ElevatorConstants.STALL_THRESHOLD_AMPS) {

      disable();
      stallDetectedAlert.set(true);

    }

  }
}
