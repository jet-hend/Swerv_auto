// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriverControl;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotMovement.SwerveDriveTrain;

public class DriveCommand extends CommandBase {
  private final SwerveDriveTrain m_drivetrainSubsystem;

  private final Double m_translationXSupplier;
  private final Double m_translationYSupplier;
  private final Double m_rotationSupplier;
  
  /** Creates a new DriveCommand. */
  public DriveCommand(SwerveDriveTrain drivetrainSubsystem, Double translationXSupplier, Double translationYSupplier, Double rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

      addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        m_translationXSupplier,
        m_translationYSupplier,
        m_rotationSupplier,
        m_drivetrainSubsystem.getGyroscopeRotation()
        )
        );
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
