// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Common.LogitechF310;
import frc.Common.ThrustMaster;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.AlgaeArm.AlgeaGrabberDefaultCommand;
import frc.robot.commands.CoralArm.CoralArmDefualtCommand;
import frc.robot.commands.CoralArm.SetCoralArmSetpoint;
import frc.robot.commands.drivetrain.DriveRotateAndYToTarget;
import frc.robot.commands.elevator.ElevatorDefaultCommand;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

import java.security.PrivateKey;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.sim.SparkAnalogSensorSim;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@SuppressWarnings("unused")
public class RobotContainer {
  // Create subsystems here
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();  
  private final AlgaeGrabberSubsystem algaeGrabber = new AlgaeGrabberSubsystem();
  private final CoralArmSubsystem coralArm = new CoralArmSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  CommandJoystick driverStick;
  CommandXboxController controller;
  CommandJoystick simStick;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {

    controller = new CommandXboxController(OperatorConstants.CO_PILOT_GAMEPAD_PORT);

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
    }

    coralArm.setDefaultCommand(new CoralArmDefualtCommand(() -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05), controller.povRight(), controller.povLeft(), driverStick.trigger(), coralArm));
    algaeGrabber.setDefaultCommand(new AlgeaGrabberDefaultCommand(() -> -controller.getRightY(), controller.povDown(), controller.povUp(), algaeGrabber));
    elevator.setDefaultCommand(new ElevatorDefaultCommand(() -> {return controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();}, elevator));
    
    // Set Default Commands here
    configureAutoCommands(coralArm);

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  } 

  private void configureBindings() {
    // Set button commands here
    /*driverStick.button(1).onTrue(new InstantCommand(drivetrain::zeroGyro));
    driverStick.button(3).onTrue(new SetElevator(elevator, ElevatorConstants.SETPOINT_3));
    driverStick.button(4).onTrue(new SetElevator(elevator, ElevatorConstants.SETPOINT_HOME));
    driverStick.button(2).onTrue(new InstantCommand(elevator::autoHome, elevator));
    driverStick.button(5).onTrue(new SetElevator(elevator, ElevatorConstants.SETPOINT_1));
    controller.button(2).onTrue(new IntakeSubsystems(intake, IntakeSubsystems)*/

    //Driver Binds
    driverStick.button(ThrustMaster.MIDDLE).whileTrue(new InstantCommand(drivetrain::lock, drivetrain));
    driverStick.button(14).onTrue(new InstantCommand(drivetrain::zeroGyro));

    driverStick.button(ThrustMaster.LEFT).whileTrue(
      new DriveRotateAndYToTarget(
        0,
        () -> Vision.getTargetRotationDegrees(0), 
        VisionConstants.LEFT_TARGET_OFFSET_M,
        () -> Vision.getTargetYMeters(VisionConstants.LEFT_TARGET_OFFSET_M),
        () -> -Math.pow(MathUtil.applyDeadband(driverStick.getY(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3),
        0.01,
        0,
        0.05,
        0,
        drivetrain
      )
    );

    driverStick.button(ThrustMaster.RIGHT).whileTrue(
      new DriveRotateAndYToTarget(
        0,
        () -> Vision.getTargetRotationDegrees(0), 
        VisionConstants.RIGHT_TARGET_OFFSET_M,
        () -> Vision.getTargetYMeters(VisionConstants.RIGHT_TARGET_OFFSET_M),
        () -> -Math.pow(MathUtil.applyDeadband(driverStick.getY(), OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3),
        0.01,
        0,
        0.05,
        0,
        drivetrain
      )
    );

    driverStick.povDown().whileTrue(new InstantCommand(() -> climber.set(ClimberConstants.CONTROL_SPEED),  climber).repeatedly().finallyDo(() -> climber.set(0)));
    driverStick.povUp().whileTrue(  new InstantCommand(() -> climber.set(-ClimberConstants.CONTROL_SPEED), climber).repeatedly().finallyDo(() -> climber.set(0)));

    //Manipulator Binds
    controller.x().onTrue(new SetCoralArmSetpoint(CoralArmConstants.HOME_SETPOINT, coralArm).alongWith(new SetElevator(elevator, ElevatorConstants.SETPOINT_HOME)));
    controller.y().onTrue(new SetCoralArmSetpoint(CoralArmConstants.L2_SETPOINT, coralArm).alongWith(new SetElevator(elevator, ElevatorConstants.SETPOINT_L2)));
    controller.b().onTrue(new SetCoralArmSetpoint(CoralArmConstants.L3_SETPOINT, coralArm).alongWith(new SetElevator(elevator, ElevatorConstants.SETPOINT_L3)));
    controller.rightBumper().onTrue(new SetCoralArmSetpoint(CoralArmConstants.L4_SETPOINT,coralArm).alongWith(new SetElevator(elevator, ElevatorConstants.SETPOINT_L3)));

    controller.a().onTrue(coralIntakeCommand);
    controller.a().onFalse(new SetCoralArmSetpoint(CoralArmConstants.HOME_SETPOINT, coralArm).alongWith(new InstantCommand(() -> coralArm.setIntake(0))));

    controller.back().onTrue(new InstantCommand(elevator::autoHome, elevator).until(() -> {return elevator.getState() != ElevatorState.HOMING;}));
  }
  
  Command coralIntakeCommand = new SequentialCommandGroup(
    new SetCoralArmSetpoint(CoralArmConstants.HOME_SETPOINT, coralArm),
    new SetElevator(elevator, ElevatorConstants.SETPOINT_HOME),
    new SetCoralArmSetpoint(CoralArmConstants.INTAKE_SETPOINT, coralArm),
    new InstantCommand(() -> coralArm.setIntake(CoralArmConstants.INTAKE_IN_SPEED), coralArm)
    .repeatedly().until(() -> coralArm.isLoaded()),
    Commands.waitSeconds(1),
    new InstantCommand(() -> coralArm.setIntake(0)),
    new SetCoralArmSetpoint(CoralArmConstants.HOME_SETPOINT, coralArm)
  );

  Command coralL4DropSequenceCommand = new SequentialCommandGroup(
    new InstantCommand(() -> coralArm.setIntake(CoralArmConstants.INTAKE_OUT_SPEED), coralArm)
    .repeatedly().until(() -> !coralArm.isLoaded()),
    Commands.waitSeconds(1),
    new InstantCommand(() -> coralArm.setIntake(0)
    )
  );

  Command coralDropSequenceCommand = new SequentialCommandGroup(
    new InstantCommand(() -> coralArm.setIntake(CoralArmConstants.INTAKE_IN_SPEED), coralArm)
    .repeatedly().until(() -> !coralArm.isLoaded()),
    Commands.waitSeconds(1),
    new InstantCommand(() -> coralArm.setIntake(0)
    )
  );

  Command L1_CORALDROP =  new SequentialCommandGroup(
    new SetCoralArmSetpoint(CoralArmConstants.L1_SETPOINT, coralArm),
    new InstantCommand(() -> coralArm.setIntake(CoralArmConstants.INTAKE_IN_SPEED), coralArm)
    .repeatedly().until(() -> !coralArm.isLoaded()),
    Commands.waitSeconds(1),
    new InstantCommand(() -> coralArm.setIntake(0)
  )
);
  

  private void configureAutoCommands(CoralArmSubsystem CoralArmSubsystem) {

    //Must add commands here before autoChooser command
    NamedCommands.registerCommand("DoNothing", new DoNothing());
    NamedCommands.registerCommand("FeedIntake", 
       coralIntakeCommand
    );
    NamedCommands.registerCommand("L1Setpoint",
       L1_CORALDROP
    );

    NamedCommands.registerCommand("ArmToHome",new SetCoralArmSetpoint(CoralArmConstants.HOME_SETPOINT, CoralArmSubsystem));
    NamedCommands.registerCommand("ArmToL1", new SetCoralArmSetpoint(CoralArmConstants.L1_SETPOINT, CoralArmSubsystem));
    NamedCommands.registerCommand("ArmToL2",new SetCoralArmSetpoint(CoralArmConstants.L2_SETPOINT, CoralArmSubsystem));
    NamedCommands.registerCommand("ArmToL3",new SetCoralArmSetpoint(CoralArmConstants.L3_SETPOINT, CoralArmSubsystem));
    NamedCommands.registerCommand("ArmToL4",new SetCoralArmSetpoint(CoralArmConstants.L4_SETPOINT, CoralArmSubsystem));
    NamedCommands.registerCommand("ElevatorHome",new SetElevator(elevator, ElevatorConstants.SETPOINT_HOME));
    NamedCommands.registerCommand("ElevatorL2",new SetElevator(elevator, ElevatorConstants.SETPOINT_L2));
    NamedCommands.registerCommand("ElevatorL3AndL4",new SetElevator(elevator, ElevatorConstants.SETPOINT_L3));
    NamedCommands.registerCommand("L4Drop", coralL4DropSequenceCommand);
    NamedCommands.registerCommand("LevelDrop", coralDropSequenceCommand);
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Autonomous Chooser", autoChooser);
    }
     
      private Command SetElevator(ElevatorSubsystem elevator2, double setpointL3) {
        throw new UnsupportedOperationException("Unimplemented method 'SetElevator'");
      }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void onEnable() {
    elevator.enable();
    algaeGrabber.enable();
    coralArm.enable();
  }

  public void onDisable() {
    elevator.disable();
    algaeGrabber.disable();
    coralArm.disable();
  }

  public void teleopPeriod(){
    
  }

}
