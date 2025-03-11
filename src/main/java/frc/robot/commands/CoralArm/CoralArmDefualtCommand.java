// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.subsystems.CoralArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralArmDefualtCommand extends Command {

  DoubleSupplier angleControl;
  Trigger intakeIn;
  Trigger intakeOut;
  Trigger driverTrigger;
  CoralArmSubsystem coralArm;
  Timer angleControlTimer = new Timer();
  public Object andThen;

  /** Creates a new CoralArmDefualtCommand. */
  public CoralArmDefualtCommand(DoubleSupplier angleControl, Trigger intakeIn, Trigger intakeOut, Trigger driverTrigger, CoralArmSubsystem coralArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleControl = angleControl;
    this.intakeIn = intakeIn;
    this.intakeOut = intakeOut;
    this.driverTrigger = driverTrigger;
    this.coralArm = coralArm;

    addRequirements(coralArm);
  }

  public CoralArmDefualtCommand(Object angleControl2, boolean b, boolean c, Object driverTrigger2,
        CoralArmSubsystem coralArm2) {
}

public CoralArmDefualtCommand(InstantCommand instantCommand) {
    //TODO Auto-generated constructor stub
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleControlTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driverTrigger.getAsBoolean()) {
      coralArm.setIntake(coralArm.getAngle() >= CoralArmConstants.DRIVER_TRIGGER_REVERSE_THRESHOLD ? CoralArmConstants.INTAKE_OUT_SPEED : CoralArmConstants.INTAKE_IN_SPEED);
    }
    else if (intakeIn.getAsBoolean()) {
      coralArm.setIntake(CoralArmConstants.INTAKE_IN_SPEED);
    }
    else if (intakeOut.getAsBoolean()) {
      coralArm.setIntake(CoralArmConstants.INTAKE_OUT_SPEED);
    }
    else {
      coralArm.setIntake(0);
    }

    //Control Angle
    coralArm.setPosition(coralArm.getSetpoint() + angleControl.getAsDouble() *  CoralArmConstants.MANUAL_CONTROL_RATE_DEG_SEC * angleControlTimer.get());
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
