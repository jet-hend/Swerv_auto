// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

//import OI
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import auto commands
import frc.robot.commands.AutonomousCommands.Blue.BlueStationOne;
import frc.robot.commands.AutonomousCommands.Blue.BlueStationThree;
import frc.robot.commands.AutonomousCommands.Blue.BlueStationTwo;
import frc.robot.commands.AutonomousCommands.Red.RedStationOne;
import frc.robot.commands.AutonomousCommands.Red.RedStationThree;
import frc.robot.commands.AutonomousCommands.Red.RedStationTwo;
import frc.robot.commands.CommonCommands.FlyWheelControl;
import frc.robot.commands.CommonCommands.IntakeControl;
import frc.robot.commands.CommonCommands.TurretAuto;
import frc.robot.commands.DriverControl.DriveCommand;
import frc.robot.commands.DriverControl.SetLift;
//import subsystems
import frc.robot.subsystems.BallMovement.Belts;
import frc.robot.subsystems.BallMovement.FlyWheel;
import frc.robot.subsystems.BallMovement.Intake;
import frc.robot.subsystems.BallMovement.Turret;
import frc.robot.subsystems.RobotMovement.Lift;
import frc.robot.subsystems.RobotMovement.SwerveDriveTrain;
import frc.robot.utilities.BeamBreaks;
import frc.robot.utilities.LimeLight;

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

  //Ball Movement Subsystems
  public final static Intake i_intake = new Intake();
  public final static Belts b_belts = new Belts();
  public final static Turret t_turret = new Turret();
  public final static FlyWheel f_flywheel = new FlyWheel();

  //RobotMovement Subsystems
  public final static Lift l_lift = new Lift();
  private final SwerveDriveTrain s_swervedrivetrain = new SwerveDriveTrain();

  /*if a file is in the Utilities folder it needs to be used in the robot multiple times in different commands*/
  public final static BeamBreaks b_beambreaks = new BeamBreaks();
  public final static LimeLight l_limelight = new LimeLight();

  //Auto Commands
    //blue
  private final BlueStationOne B_1 = new BlueStationOne();
  private final BlueStationTwo B_2 = new BlueStationTwo();
  private final BlueStationThree B_3 = new BlueStationThree();
    //red
  private final RedStationOne R_1 = new RedStationOne();
  private final RedStationTwo R_2 = new RedStationTwo();
  private final RedStationThree R_3 = new RedStationThree();

  //Driver Commands


  //Button Box
  public static Joystick BottonBox = new Joystick(Constants.OI.BOTTON_BOX_PORT);

  public static JoystickButton REDR = new JoystickButton(BottonBox, 2);
  public static JoystickButton REDL = new JoystickButton(BottonBox, 1);
  public static JoystickButton BLUER = new JoystickButton(BottonBox, 4);
  public static JoystickButton BLUEL = new JoystickButton(BottonBox, 3);
  public static JoystickButton YELLOWR = new JoystickButton(BottonBox, 5);
  public static JoystickButton YELLOWL = new JoystickButton(BottonBox, 6);
  public static JoystickButton GREENR = new JoystickButton(BottonBox, 7);
  public static JoystickButton GREENL = new JoystickButton(BottonBox, 8);
  
//Controller
  public static Joystick XboxC = new Joystick(Constants.OI.JOYSTICK_PORT);

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

  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_swervedrivetrain.setDefaultCommand(new DriveCommand(
        s_swervedrivetrain,
        -modifyAxis(XboxC.getRawAxis(1)),
        -modifyAxis(XboxC.getRawAxis(2)),
        -modifyAxis(XboxC.getRawAxis(3))
      )
    );
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Map Buttons and such to commands

    up.whenPressed(new SetLift(l_lift, "up"));
    down.whenPressed(new SetLift(l_lift, "down"));

    RB.whenPressed(new TurretAuto(l_limelight, t_turret, true));
    LB.whenPressed(new TurretAuto(l_limelight, t_turret, false));

    AB.whenPressed(new IntakeControl(i_intake, 0.6, true));
    BB.whenPressed(new IntakeControl(i_intake, 0.0, false));

    XB.whenPressed(new TurretAuto(l_limelight, t_turret, false));
    XB.whenPressed(new FlyWheelControl(f_flywheel, b_belts, ControlMode.PercentOutput, 0.6, 0.4));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   static //gets DriverStation info
  DriverStation.Alliance color = DriverStation.getAlliance();
  static int station = DriverStation.getLocation();
  static String Color = color.toString();

  public Command getAutonomousCommand() {
    //run in autonomous
    Command AutoCommand = null;
    if (color == DriverStation.Alliance.Blue) {
      switch (station) {
        case 1:
          AutoCommand = B_1;
          break;
        case 2:
          AutoCommand = B_2;
          break;
        case 3:
          AutoCommand = B_3;
          break;
        default:
          AutoCommand = null;
          break;
      }
    }
    if (color == DriverStation.Alliance.Red) {
      switch (station) {
        case 1:
          AutoCommand = R_1;
          break;
        case 2:
          AutoCommand = R_2;
          break;
        case 3:
          AutoCommand = R_3;
          break;
        default:
          AutoCommand = null;
          break;
      }
    }
    return AutoCommand;
  }

}
