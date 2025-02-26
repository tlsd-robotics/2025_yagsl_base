// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefaultCommand extends Command {

  DoubleSupplier controlAxis;
  ElevatorSubsystem elevator;
  Timer controlTimer = new Timer();

  /** Creates a new ElevatorDefaultCommand. */
  public ElevatorDefaultCommand(DoubleSupplier controlAxis, ElevatorSubsystem elevator) {
    
    this.controlAxis = controlAxis;
    this.elevator = elevator;

    addRequirements(elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controlTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevator.setPosition(elevator.getSetpoint() + controlAxis.getAsDouble() * ElevatorConstants.MANUAL_CONTROL_RATE_M_S * controlTimer.get());
    controlTimer.restart();

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
