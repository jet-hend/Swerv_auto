// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import auto commands
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveDriveTrain;
//import subsystem
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final static SwerveDriveTrain s_swervedrivetrain = new SwerveDriveTrain();

  //Button Box
  // public static Joystick BottonBox = new Joystick(Constants.OI.BOTTON_BOX_PORT);

  // public static JoystickButton REDR = new JoystickButton(BottonBox, 2);
  // public static JoystickButton REDL = new JoystickButton(BottonBox, 1);
  // public static JoystickButton BLUER = new JoystickButton(BottonBox, 4);
  // public static JoystickButton BLUEL = new JoystickButton(BottonBox, 3);
  // public static JoystickButton YELLOWR = new JoystickButton(BottonBox, 5);
  // public static JoystickButton YELLOWL = new JoystickButton(BottonBox, 6);
  // public static JoystickButton GREENR = new JoystickButton(BottonBox, 7);
  // public static JoystickButton GREENL = new JoystickButton(BottonBox, 8);
  
//Controller
  public static XboxController XboxC = new XboxController(Constants.OI.JOYSTICK_PORT);

  public int LX = XboxController.Axis.kLeftX.value;
  public int LY = XboxController.Axis.kLeftY.value;
  public int RX = XboxController.Axis.kRightX.value;
  public int RY = XboxController.Axis.kRightY.value;

  public static JoystickButton AB = new JoystickButton(XboxC, 1);
  public static JoystickButton BB = new JoystickButton(XboxC, 2);
  public static JoystickButton XB = new JoystickButton(XboxC, 3);
  public static JoystickButton YB = new JoystickButton(XboxC, 4);
  public static JoystickButton LB = new JoystickButton(XboxC, 5);
  public static JoystickButton RB = new JoystickButton(XboxC, 6);
  public static JoystickButton HB = new JoystickButton(XboxC, 7);
  public static JoystickButton ZB = new JoystickButton(XboxC, 8);
  public static JoystickButton LJB = new JoystickButton(XboxC, 9);
  public static JoystickButton RJB = new JoystickButton(XboxC, 10);
  public static POVButton up = new POVButton(XboxC, 0);
  public static POVButton right = new POVButton(XboxC, 90);
  public static POVButton down = new POVButton(XboxC, 180);
  public static POVButton left = new POVButton(XboxC, 270);

  //Swerve Drive input Axis setup (deadband)
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  static double outp;
  public static double EXPout(double input) {
    return outp = Math.pow(input, 3);
  }

  public static Double modifyAxis(Double value) {
    // Deadband
    value = deadband(value, 0.08);
    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    s_swervedrivetrain.setDefaultCommand(new DriveCommand(s_swervedrivetrain, XboxC, RX, RY, LX));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Map Buttons and such to commands
    BB.whenPressed(new DriveCommand(s_swervedrivetrain, XboxC, RX, RY, LX));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    //gets DriverStation info
  public Command getAutonomousCommand() {
    //run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.SwerveDrive.m_kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)),
      List.of(
        new Translation2d(1,0),
        new Translation2d(1,-1)
      ),
      new Pose2d(2,-1, Rotation2d.fromDegrees(180)), 
      trajectoryConfig
    );

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      s_swervedrivetrain::getPose,
      Constants.SwerveDrive.m_kinematics,
      xController,
      yController,
      thetaController,
      s_swervedrivetrain::setModuleStates,
      s_swervedrivetrain
    );
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> s_swervedrivetrain.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> s_swervedrivetrain.stopModules())
    );
  }
}
