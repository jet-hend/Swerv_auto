// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallMovement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TalonFX TurretM = new TalonFX(Constants.Turret.Turret_Motor_ID);

  //set/outs

  //set the turret to percent out put limiting the motor between 12v and 0.0v
  public void setTurretPO (double PO) {
    TurretM.set(ControlMode.PercentOutput, PO);
  }

  //sets the turret to Position control //takes in a position to set relitive to the encoder value
  public void setTurretPOS (double POS) {
    TurretM.set(ControlMode.Position, POS);
  }

  //get/returns

  //returns the turret encoder Pos
  public double getTurretEncoder () {
    return TurretM.getSelectedSensorPosition();
  }

}
