// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Common.LogitechF310;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  // Create subsystems here
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();  

  CommandJoystick driverStick;
  LogitechF310 controller;
  CommandJoystick simStick;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Handle simulation
    if (RobotBase.isSimulation()) {
      simStick = new CommandJoystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
      drivetrain.setDefaultCommand(
        drivetrain.defaultDriveCommand(
          () -> -Math.pow(MathUtil.applyDeadband(simStick.getY(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3), 
          () -> -Math.pow(MathUtil.applyDeadband(simStick.getX(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3),
          () -> -Math.pow(MathUtil.applyDeadband(driverStick.getZ(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3)
        ));
    } else {
      driverStick = new CommandJoystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
      // controller = new LogitechF310(OperatorConstants.CO_PILOT_GAMEPAD_PORT);
      drivetrain.setDefaultCommand(
        drivetrain.defaultDriveCommand(
          () -> -Math.pow(MathUtil.applyDeadband(driverStick.getY(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3), 
          () -> -Math.pow(MathUtil.applyDeadband(driverStick.getX(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3),
          () -> -Math.pow(MathUtil.applyDeadband(driverStick.getZ(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3)
        ));

      configureBindings();

      elevator.enable();
    }

    // Set Default Commands here

    
    configureAutoCommands();
  } 

  private void configureBindings() {
    // Set button commands here
    driverStick.button(1).onTrue(new InstantCommand(drivetrain::zeroGyro));
    driverStick.button(3).onTrue(new SetElevator(elevator, ElevatorConstants.SETPOINT_3));
    driverStick.button(4).onTrue(new SetElevator(elevator, ElevatorConstants.SETPOINT_HOME));
    driverStick.button(2).onTrue(new InstantCommand(elevator::autoHome));
    driverStick.button(5).onTrue(new SetElevator(elevator, ElevatorConstants.SETPOINT_1));
  } 

  private void configureAutoCommands() {
    autoChooser = AutoBuilder.buildAutoChooser();

    // Add Auto Commands here
    NamedCommands.registerCommand("DoNothing", new DoNothing());

    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
