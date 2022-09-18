// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallMovement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //makes the intake motor
  private TalonFX IntakeMotor = new TalonFX(Constants.intake.Intake_Motor_ID);
  //sets the intake motor
  public void setIntakeMotor (double PO) {
    IntakeMotor.set(ControlMode.PercentOutput, PO);
  }

  //gets current draw on intake motor
  public double getIntakeCurrentDraw () {
    return IntakeMotor.getStatorCurrent();
  }
  //gets current input on intake motor
  public double getIntakeCurrentInput () {
    return IntakeMotor.getSupplyCurrent();
  }
}
