// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveTrain;

public class DriveCommand extends CommandBase {
  private final SwerveDriveTrain m_drivetrainSubsystem;

  //private Joystick controller;
  private int m_translationXSupplier;
  private int m_translationYSupplier;
  private int m_rotationSupplier;

  private double Dead = 0.08;

  private XboxController controller;
  
  /** Creates a new DriveCommand. */
  public DriveCommand(SwerveDriveTrain drivetrainSubsystem, XboxController controller, int X, int Y, int R) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    //this.controller = controller;
    addRequirements(drivetrainSubsystem);

    this.controller = controller;
    this.m_translationYSupplier = Y;
    this.m_translationXSupplier = X;
    this.m_rotationSupplier = R;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.zeroGyroscope();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xAxis = -controller.getRawAxis(m_translationYSupplier);
    double yAxis = -controller.getRawAxis(m_translationXSupplier);
    double rAxis = -controller.getRawAxis(m_rotationSupplier);

    xAxis = RobotContainer.modifyAxis(-xAxis)* Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND;
    yAxis = RobotContainer.modifyAxis(-yAxis)* Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND;
    rAxis = RobotContainer.modifyAxis(-rAxis)* Constants.SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    if(xAxis >= Dead || xAxis <= -Dead) {
    } else {
      xAxis = 0.0;
    }

    if(yAxis >= Dead || yAxis <= -Dead) {
    } else {
      yAxis = 0.0;
    }

    if(rAxis >= Dead || rAxis <= -Dead) {
    } else {
      rAxis = 0.0;
    }

    // m_drivetrainSubsystem.drive(
    //   ChassisSpeeds.fromFieldRelativeSpeeds(
    //   xAxis, 
    //   yAxis,
    //   rAxis, 
    //   m_drivetrainSubsystem.getGyroscopeRotation())
    // );

    ChassisSpeeds m_chassisSpeeds;
    m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xAxis, 
      yAxis,
      rAxis, 
      m_drivetrainSubsystem.getGyroscopeRotation()
    );

    SwerveModuleState[] states = Constants.SwerveDrive.m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    m_drivetrainSubsystem.setModuleStates(states);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
