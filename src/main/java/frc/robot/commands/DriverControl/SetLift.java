// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriverControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotMovement.Lift;
import frc.robot.utilities.AirSystem;

public class SetLift extends CommandBase {
  private Lift l_lift;
  private String state;
  private AirSystem a_airsystem;
  /** Creates a new SetLift. */
  public SetLift(Lift l_lift, String state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.l_lift = l_lift;
    this.state = state;
    addRequirements(l_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l_lift.setLiftMotorPOS(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case "up":
        a_airsystem.setLiftSolenoid(true);
        try {
          Thread.sleep(500);
        } catch (InterruptedException e1) {
          e1.printStackTrace();
        }
        l_lift.setLiftMotorPOS(Constants.JunkDrawer.Lift_Max);
        break;
      case "down":
        a_airsystem.setLiftSolenoid(false);
        try {
          Thread.sleep(500);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        l_lift.setLiftMotorPOS(Constants.JunkDrawer.Lift_Min);
        break;
      default:
      a_airsystem.setLiftSolenoid(false);
        l_lift.setLiftMotorPOS(0);
        break;
    }
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
