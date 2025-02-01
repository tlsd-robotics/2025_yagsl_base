// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private File swerveConfigDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  private SwerveDrive drive;
  private RobotConfig config;

  public DrivetrainSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      drive = new SwerveParser(swerveConfigDirectory).createSwerveDrive(DrivetrainConstants.MAX_VELOCITY);
      drive.setHeadingCorrection(!RobotBase.isSimulation());
    }
    catch (IOException e) {
      throw new RuntimeException("Could not find swerve drive files!!");
    }

    try {
      config = RobotConfig.fromGUISettings();
      configurePathplanner();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  public Command defaultDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
    return run (
      () -> {
        drive.driveFieldOriented (
          new ChassisSpeeds (
            xSpeed.getAsDouble() * drive.getMaximumChassisVelocity(),
            ySpeed.getAsDouble() * drive.getMaximumChassisVelocity(),
            zRotation.getAsDouble() * drive.getMaximumChassisAngularVelocity()
          )
        );
      }
    );
  }

  public Command trajectoryDrive(ChassisSpeeds speed) {
    return run (
      () -> {
        drive.drive(speed);
      }
    );
  }

  public Pose2d getPose2d() {
    return drive.getPose();
  }

  public void resetPose2d(Pose2d pose) {
    drive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drive.getRobotVelocity();
  }

  public void configurePathplanner() {
    AutoBuilder.configure(
            this::getPose2d, // Robot pose supplier
            this::resetPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, 
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
  
  public void zeroGyro() {
    drive.zeroGyro();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    drive.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drive.updateOdometry();
  }
}
