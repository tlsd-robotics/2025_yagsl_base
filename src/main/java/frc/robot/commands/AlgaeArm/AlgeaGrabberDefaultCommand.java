// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeArm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeGrabberConstants;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgeaGrabberDefaultCommand extends Command {

  DoubleSupplier angleControl;
  Trigger intakeIn;
  Trigger intakeOut;
  AlgaeGrabberSubsystem AlgaeGrabber;
  Timer angleControlTimer = new Timer();

  /** Creates a new AlgaeGrabberDefualtCommand. */
  public AlgeaGrabberDefaultCommand(DoubleSupplier angleControl, Trigger intakeIn, Trigger intakeOut, AlgaeGrabberSubsystem AlgaeGrabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleControl = angleControl;
    this.intakeIn = intakeIn;
    this.intakeOut = intakeOut;
    this.AlgaeGrabber = AlgaeGrabber;

    addRequirements(AlgaeGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleControlTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeIn.getAsBoolean()) {
      AlgaeGrabber.setIntake(AlgaeGrabberConstants.INTAKE_IN_SPEED);
    }
    else if (intakeOut.getAsBoolean()) {
      AlgaeGrabber.setIntake(AlgaeGrabberConstants.INTAKE_OUT_SPEED);
    }
    else {
      AlgaeGrabber.setIntake(0);;
    }

    //Control Angle
    AlgaeGrabber.setPosition(AlgaeGrabber.getSetpoint() + angleControl.getAsDouble() * AlgaeGrabberConstants.MANUAL_CONTROL_RATE_DEG_SEC * angleControlTimer.get());
    angleControlTimer.restart();
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
