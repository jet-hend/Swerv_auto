// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.RobotMovement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  public Lift() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LiftMotor.config_kP(0, 1.0);
    LiftMotor.config_kI(0, 0.0);
    LiftMotor.config_kD(0, 0.0);
  }

  private TalonFX LiftMotor = new TalonFX(Constants.JunkDrawer.Lift_Motor_ID);

  //sets lift motor to use PID for position
  public void setLiftMotorPOS (double POS) {
    LiftMotor.set(ControlMode.Position, POS);
  }

  public void setLiftMotorPO (double PO) {
    LiftMotor.set(ControlMode.PercentOutput, PO);
  }
}
