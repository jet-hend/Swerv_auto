// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AirSystem extends SubsystemBase {
  /** Creates a new AirSystem. */
  public AirSystem() {
    pHub.makeCompressor().enableAnalog(10, 120);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public PneumaticHub pHub = new PneumaticHub(Constants.AirSystem.Pneumatic_Hub_ID);
  //  public DoubleSolenoid m_doubleSolenoid = pHub.makeDoubleSolenoid(14, 0);
  //  public DoubleSolenoid m_doubleSolenoidL = pHub.makeDoubleSolenoid(5, 6);
  private Solenoid intakeSolenoid = pHub.makeSolenoid(1);
  private Solenoid liftSolenoid = pHub.makeSolenoid(2);

  public void setLiftSolenoid (boolean state) {
    liftSolenoid.set(state);
  }

  public void setIntakeSolenoid (boolean state) {
    intakeSolenoid.set(state);
  }
  
}
