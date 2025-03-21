// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveRotateAndYToTarget extends Command {
  /** Creates a new DriveRotateToTarget. */

  PIDController drivePID;
  PIDController anglePID;
  DrivetrainSubsystem drivetrain;
  DoubleSupplier angleInput;
  DoubleSupplier translationInput;
  DoubleSupplier xSpeed;
  double angleTarget;
  double translationTarget;

  public DriveRotateAndYToTarget(double angleTarget, DoubleSupplier angleInput, double translationTarget, DoubleSupplier tranlationInput,
                             DoubleSupplier xSpeed, double Angle_PID_P, double Angle_PID_D, double Translation_PID_P, double Translation_PID_D, DrivetrainSubsystem drivetrain) {

    this.angleTarget = angleTarget;
    this.angleInput = angleInput;
    this.translationTarget = translationTarget;
    this.translationInput = tranlationInput;
    this.xSpeed = xSpeed;
    this.anglePID = new PIDController(Angle_PID_P, 0, Angle_PID_D);
    this.drivePID = new PIDController(Translation_PID_P, 0, Translation_PID_D);
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.reset();
    drivePID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.drive(
      new ChassisSpeeds (
        xSpeed.getAsDouble() * drivetrain.getMaxVelocityMS(),
        drivePID.calculate(translationInput.getAsDouble(), translationTarget),
        anglePID.calculate(angleInput.getAsDouble(), angleTarget)
      )
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
