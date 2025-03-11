// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Common.ConstraintClasses.RangeConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //=======================================================================================================================
  //================================               Operator Constants              ========================================
  //=======================================================================================================================
  public static class OperatorConstants {
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int CO_PILOT_GAMEPAD_PORT = 1;
    public static final double DRIVER_JOYSTICK_DEADBAND = 0.1;
  }

  //=======================================================================================================================
  //===============================               Drivetrain Constants              =======================================
  //=======================================================================================================================
  public static class DrivetrainConstants {
    public static final double MAX_VELOCITY = 4.5;
  }

  //=======================================================================================================================
  //==============================              Superstructure Constants              =====================================
  //=======================================================================================================================
  public static class SuperstructureConstants {
    public static final int PDP_ID = 1;
  }

  //=======================================================================================================================
  //=================================               Vision Constants              =========================================
  //=======================================================================================================================
  public static final class VisionConstants {
    public static final Translation3d ROBOT_TO_CAM_TRANSLATION = new Translation3d(Units.inchesToMeters(11.5), 0.0, Units.inchesToMeters(4)); // X = forward, Y = left, Z = up
    public static final Rotation3d ROBOT_TO_CAM_ROTATION = new Rotation3d(0.0, Units.degreesToRadians(44.5), 0.0);
    public static final String ARM_CAM_NAME = "TopUsb";
    public static final String INTAKE_CAM_NAME = "BottomUsb";
  }

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.0001, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.00001, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }



  public static final class ElevatorConstants {

    public static final int ELEVATOR_MOTOR_ID = 35;
    public static final boolean ELEVATOR_MOTOR_INVERTED = false;

    public static final double STALL_THRESHOLD_AMPS = 40.0; 
    public static final int ELEVATOR_MOTOR_SMART_CURRENT_LIMIT = 80;
    //OG Limit 40
    public static final double VELOCITY_TOLERANCE_MS = 0.001;

    //Autohome constants
    public static final double AUTOHOME_OUTPUT_PERCENT = 0.1;
    public static final double AUTOHOME_WAIT_TIME_SEC = 0.5;

    //Setpoints
    public static final double SETPOINT_L2 = Units.inchesToMeters(10);
    public static final double SETPOINT_L3 = Units.inchesToMeters(25);
    public static final double SETPOINT_HOME = 0;
    //12
    //PID Controller Constants
    public static final double PID_P = 15.0;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.0;

    public static final double SETPOINT_TOLERANCE = 0.01; //1 cm

    //FeedForward Controller Constants

    //System Constants
    public static final double ELEVATOR_MAX_HEIGHT_M = Units.inchesToMeters(26.5);

    public static final double ELEVATOR_ALLOWED_ACCEL_MSS = 0.8;
    public static final double MOTOR_GEAR_RATIO = 15.0;
    public static final double PINION_GEAR_DIAMETER_M = Units.inchesToMeters(1);
    public static final double ELEVATOR_MOTOR_RPS_MAX = 5700.0/60.0;
    public static final double ELEVATOR_MAX_SPEED_MS = (ELEVATOR_MOTOR_RPS_MAX / MOTOR_GEAR_RATIO) * PINION_GEAR_DIAMETER_M * Math.PI;
    //Converts the motor revolutions into meters of elevator travel:
    public static final double POSITION_CONVERSION_FACTOR = (PINION_GEAR_DIAMETER_M * Math.PI) / MOTOR_GEAR_RATIO;
    //Converts motor RPM into elevator velocity in m/s      Shaft revs per sec        * Distance Per Revolution
    public static final double VELOCITY_CONVERSION_FACTOR = (1 / (MOTOR_GEAR_RATIO * 60)) * PINION_GEAR_DIAMETER_M * Math.PI;
    
    //Manual Elevator control command
    public static final double MANUAL_CONTROL_RATE_M_S = 0.1;
  }


  public static final class AlgaeGrabberConstants {

    public static final int ANGLE_MOTOR_ID = 33;
    public static final int INTAKE_MOTOR_ID = 31;
    public static final boolean ANGLE_MOTOR_INVERTED = false;
    public static final boolean INTAKE_MOTOR_INVERTED = false;
    public static final int ANGLE_MOTOR_SMART_CURRENT_LIMIT = 40;
    public static final int INTAKE_MOTOR_SMART_CURRENT_LIMIT = 40;
    public static final int ABSOLUTE_ENCODER_PORT = 4;
    public static final double ABSOLUTE_ENCODER_OFFSET_DEGREES = 42.17;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = false;


    public static final RangeConstraint ANGLE_RANGE_DEGREES = new RangeConstraint(-20.0, 60.0);

    public static final double ANGLE_PID_P = 0.016;
    public static final double ANGLE_PID_I = 0.00001;
    public static final double ANGLE_PID_D = 0;

    public static final double FF_KS = 0;
    public static final double FF_KG = 0;
    public static final double FF_KV = 0;

    public static final double ANGLE_SETPOINT_TOLERANCE_DEGREES = 1.0;

    public static final double MAX_ANGULAR_VELOCITY_DEG_SEC = 90.0;
    public static final double MAX_PROFILED_ANGULAR_ACCELERATION_DEG_SEC_SEC = 90.0;

    public static final double INTAKE_IN_SPEED = -0.6;
    public static final double INTAKE_OUT_SPEED = 0.3;
    
    public static final double MANUAL_CONTROL_RATE_DEG_SEC = 80.0;

  }

  public static final class CoralArmConstants {

    public static final int ANGLE_MOTOR_ID = 32;
    public static final int INTAKE_MOTOR_ID = 34;
    public static final boolean ANGLE_MOTOR_INVERTED = true;
    public static final boolean INTAKE_MOTOR_INVERTED = false;
    public static final int ANGLE_MOTOR_SMART_CURRENT_LIMIT = 40;
    public static final int INTAKE_MOTOR_SMART_CURRENT_LIMIT = 20;
    public static final int ABSOLUTE_ENCODER_PORT = 5;
    public static final double ABSOLUTE_ENCODER_OFFSET_DEGREES = -2.8;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = true;


    public static final RangeConstraint ANGLE_RANGE_DEGREES = new RangeConstraint(-116, 80);

    public static final double UP_SETPOINT = 70;
    public static final double HOME_SETPOINT = -80;
    public static final double INTAKE_SETPOINT = -116;

    public static final double ANGLE_PID_P = 0.015;
    public static final double ANGLE_PID_I = 0.001;
    public static final double ANGLE_PID_D = 0.0;

    public static final double FF_KS = 0;
    public static final double FF_KG = 0;
    public static final double FF_KV = 0;

    public static final double ANGLE_SETPOINT_TOLERANCE_DEGREES = 1.0;

    public static final double MAX_ANGULAR_VELOCITY_DEG_SEC = 140;
    public static final double MAX_PROFILED_ANGULAR_ACCELERATION_DEG_SEC_SEC = 140;

    public static final double DRIVER_TRIGGER_REVERSE_THRESHOLD = 0;

    public static final double INTAKE_OUT_SPEED = 1.0;
    public static final double INTAKE_IN_SPEED = -1.0;

    public static final double MANUAL_CONTROL_RATE_DEG_SEC = 90.0;
  }

  public static class ClimberConstants {

    public static final int MOTOR_ID = 30;
    public static final int ABSOLUTE_ENCODER_PORT = 6;
    public static final boolean MOTOR_INVERTED = true;
    public static final boolean ENCODER_IVNERTED = true;
    
    public static final double ABSOLUTE_ENCODER_OFFSET = 197.3;

    public static final RangeConstraint ANGLE_RANGE_DEGREES = new RangeConstraint(0, 220);

    public static final double CONTROL_SPEED = 0.3;

    public static final int SMART_CURRENT_LIMIT_AMPS = 40;

  }
}
