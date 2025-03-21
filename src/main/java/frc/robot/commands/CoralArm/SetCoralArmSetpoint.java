// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralArmSetpoint extends Command {
  /** Creates a new SetCoralArm. */

  CoralArmSubsystem coralArm;
  double setpoint;

  public SetCoralArmSetpoint(double setpoint, CoralArmSubsystem coralArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
    this.coralArm = coralArm;

    addRequirements(coralArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralArm.setProfiled(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralArm.atSetpoint();
  }
}
