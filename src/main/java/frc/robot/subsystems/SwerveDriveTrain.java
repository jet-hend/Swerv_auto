// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveTrain extends SubsystemBase {

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final static SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.SwerveDrive.m_kinematics, new Rotation2d(0));

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  /** Creates a new SwerveDriveTrain. */
  public SwerveDriveTrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
      // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
      tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
      // This can either be STANDARD or FAST depending on your gear configuration
      Mk4iSwerveModuleHelper.GearRatio.L1,
      // This is the ID of the drive motor
      Constants.SwerveDrive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      // This is the ID of the steer motor
      Constants.SwerveDrive.FRONT_LEFT_MODULE_STEER_MOTOR,
      // This is the ID of the steer encoder
      Constants.SwerveDrive.FRONT_LEFT_MODULE_STEER_ENCODER,
      // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
      Constants.SwerveDrive.FRONT_LEFT_MODULE_STEER_OFFSET);

      // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
      tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
      Mk4iSwerveModuleHelper.GearRatio.L1,
      Constants.SwerveDrive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.SwerveDrive.FRONT_RIGHT_MODULE_STEER_MOTOR,
      Constants.SwerveDrive.FRONT_RIGHT_MODULE_STEER_ENCODER,
      Constants.SwerveDrive.FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
      tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
      Mk4iSwerveModuleHelper.GearRatio.L1,
      Constants.SwerveDrive.BACK_LEFT_MODULE_DRIVE_MOTOR,
      Constants.SwerveDrive.BACK_LEFT_MODULE_STEER_MOTOR,
      Constants.SwerveDrive.BACK_LEFT_MODULE_STEER_ENCODER,
      Constants.SwerveDrive.BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
      tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
      Mk4iSwerveModuleHelper.GearRatio.L1,
      Constants.SwerveDrive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.SwerveDrive.BACK_RIGHT_MODULE_STEER_MOTOR,
      Constants.SwerveDrive.BACK_RIGHT_MODULE_STEER_ENCODER,
      Constants.SwerveDrive.BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  public void zeroGyroscope() {;
    m_navx.zeroYaw();
    System.out.println("zeroed");
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                }
      // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
      return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
      m_chassisSpeeds = chassisSpeeds;
    }

    public SwerveModuleState getStates(SwerveModule name) {
      return new SwerveModuleState(name.getDriveVelocity(), new Rotation2d(name.getSteerAngle()));
    }

    public Pose2d getPose() {
      return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
      odometer.resetPosition(pose, getGyroscopeRotation());
    }

    public void stopModules() {
      drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  public void setModuleStates(SwerveModuleState[] States) {
    SwerveDriveKinematics.desaturateWheelSpeeds(States, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
    SwerveModuleState[] states = Constants.SwerveDrive.m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    m_frontLeftModule.set(states[0].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[3].angle.getRadians());
  }


  @Override
  public void periodic() {
    odometer.update(getGyroscopeRotation(), getStates(m_frontLeftModule), getStates(m_frontRightModule), getStates(m_backLeftModule), getStates(m_backRightModule));
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    // This method will be called once per scheduler run
    // SwerveModuleState[] states = Constants.SwerveDrive.m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    // m_frontLeftModule.set(states[0].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[0].angle.getRadians());
    // m_frontRightModule.set(states[1].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[1].angle.getRadians());
    // m_backLeftModule.set(states[2].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[2].angle.getRadians());
    // m_backRightModule.set(states[3].speedMetersPerSecond / Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveDrive.MAX_VOLTAGE, states[3].angle.getRadians());
  }
}
