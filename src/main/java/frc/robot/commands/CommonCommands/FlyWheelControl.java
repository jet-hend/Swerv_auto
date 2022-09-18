// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommonCommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovement.Belts;
import frc.robot.subsystems.BallMovement.FlyWheel;

public class FlyWheelControl extends CommandBase {
  private FlyWheel f_flywheel;
  private Belts b_belts;
  private ControlMode CM;
  private double Svalue;
  private double Kvalue;
  private boolean FIN = false;
  /** Creates a new FlyWheelControl. */
  public FlyWheelControl(FlyWheel f_flywheel, Belts b_belts, ControlMode CM, double Svalue, double Kvalue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.f_flywheel = f_flywheel;
    this.b_belts = b_belts;
    this.CM = CM;
    this.Svalue = Svalue;
    this.Kvalue = Kvalue;
    addRequirements(f_flywheel, b_belts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Svalue > 0) {
      f_flywheel.setShootMotors(CM, Svalue);
      try {
        Thread.sleep(500);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      f_flywheel.setKickerMotor(Kvalue);
      b_belts.setbelt1(0.3);
      b_belts.setbelt2(0.3);
      FIN = true;
    }
    if (Svalue < 0) {
      f_flywheel.setShootMotors(CM, -0.5);
      f_flywheel.setKickerMotor(-0.5);
      b_belts.setbelt1(-0.3);
      b_belts.setbelt2(-0.3);
      FIN = true;
    }
    if (Svalue == 0) {
      f_flywheel.setKickerMotor(0);
      b_belts.setbelt1(0.0);
      b_belts.setbelt2(0.0);
      f_flywheel.setShootMotors(ControlMode.PercentOutput, 0.0);
      FIN = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return FIN;
  }
}
