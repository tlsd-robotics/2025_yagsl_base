// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

    public static final double STALL_THRESHOLD_AMPS = 40.0; //TODO: tune this

    public static final double VELOCITY_TOLERANCE_MS = 0.001;

    //Autohome constants
    public static final double AUTOHOME_OUTPUT_PERCENT = 0.1;
    public static final double AUTOHOME_WAIT_TIME_SEC = 0.5;

    //Setpoints
    public static final double SETPOINT_1 = Units.inchesToMeters(5);
    public static final double SETPOINT_2 = Units.inchesToMeters(15);
    public static final double SETPOINT_3 = Units.inchesToMeters(26);
    public static final double SETPOINT_HOME = 0;

    //PID Controller Constants
    public static final double PID_P = 12.0;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.0;

    public static final double SETPOINT_TOLERANCE = 0.01; //1 cm

    //FeedForward Controller Constants

    //System Constants
    public static final double ELEVATOR_MAX_HEIGHT_M = Units.inchesToMeters(26.5);

    public static final double ELEVATOR_ALLOWED_ACCEL_MSS = 1.0;
    public static final double MOTOR_GEAR_RATIO = 25.0;
    public static final double PINION_GEAR_DIAMETER_M = Units.inchesToMeters(1);
    public static final double ELEVATOR_MOTOR_RPS_MAX = 5700.0/60.0;
    public static final double ELEVATOR_MAX_SPEED_MS = (ELEVATOR_MOTOR_RPS_MAX / MOTOR_GEAR_RATIO) * PINION_GEAR_DIAMETER_M * Math.PI;
    //Converts the motor revolutions into meters of elevator travel:
    public static final double POSITION_CONVERSION_FACTOR = (PINION_GEAR_DIAMETER_M * Math.PI) / MOTOR_GEAR_RATIO;
    //Converts motor RPM into elevator velocity in m/s      Shaft revs per sec        * Distance Per Revolution
    public static final double VELOCITY_CONVERSION_FACTOR = (1 / (MOTOR_GEAR_RATIO * 60)) * PINION_GEAR_DIAMETER_M * Math.PI;
  }
}
