// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralArmConstants;

public class IntakeSubsystem extends SubsystemBase {

 private SparkMax intakeMotor = new SparkMax(CoralArmConstants.INTAKE_MOTOR_ID,MotorType.kBrushless);
 private SparkMaxConfig intakeConfig = new SparkMaxConfig();
 public PIDController intakePIDController = new PIDController(0.0001, 0, 0);
}

