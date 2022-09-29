// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //IO ports
    public static final class OI {
        public static final int BOTTON_BOX_PORT = 1;
        public static final int JOYSTICK_PORT = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class SwerveDrive {

        public final static double MAX_VOLTAGE = 12.0;
        /**
        * The left-to-right distance between the drivetrain wheels
        *
        * Should be measured from center to center.
        */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6604; // Measure and set trackwidth
        /**
        * The front-to-back distance between the drivetrain wheels.
        *
        * Should be measured from center to center.
        */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6604; // Measure and set wheelbase

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.SwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
        new Translation2d(Constants.SwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.SwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
        new Translation2d(Constants.SwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.SwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
        new Translation2d(-Constants.SwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.SwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
        new Translation2d(-Constants.SwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.SwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0));
        ////////////////////////////////////////////////////////////////////////////////////////////////

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; // front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(5.44); // Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; // front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; // front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(269.4); // Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; // back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; // back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(339.96); // Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; // back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; // back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; // back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(4.48); // Measure and set back right steer offset
    }
}

