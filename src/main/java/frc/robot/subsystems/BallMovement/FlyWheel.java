// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallMovement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlyWheel extends SubsystemBase {
  /** Creates a new FlyWheel. */
  public FlyWheel() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //makes FlyWheel motors
  private TalonFX RS = new TalonFX(Constants.Shooter.Shoot_Motor_Right_ID);
  private TalonFX LS = new TalonFX(Constants.Shooter.Shoot_Motor_Left_ID);
  
  //makes kicker motor
  private TalonFX Kicker = new TalonFX(Constants.Shooter.Kicker_Motor_ID);


  //sets the FlyWheel motors //Right motor follows The Left motor
  public void setShootMotors (ControlMode CM, double speed) {
    LS.set(CM, speed);
    RS.setInverted(true);
    RS.follow(RS);
  }

  //sets the kicker motor
  public void setKickerMotor (double PO) {
    Kicker.set(ControlMode.PercentOutput, PO);
  }
}
