// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallMovement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Belts extends SubsystemBase {
  /** Creates a new Belts. */
  public Belts() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //makes the Lower belts motor
  private TalonFX Belt1 = new TalonFX(Constants.belts.Bottom_Belts_Motor_ID);
  //sets the lower belts motor
  public void setbelt1 (double PO) {
    Belt1.set(ControlMode.PercentOutput, PO);
  }
  //makes the upper belts motor
  private TalonFX Belt2 = new TalonFX(Constants.belts.Upper_Belts_Motor_ID);
  //sets the upper belts motor
  public void setbelt2 (double PO) {
    Belt2.set(ControlMode.PercentOutput, PO);
  }
}
