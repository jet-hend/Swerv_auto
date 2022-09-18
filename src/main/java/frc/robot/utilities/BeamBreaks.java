// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreaks extends SubsystemBase {
  /** Creates a new BeamBreaks. */
  public BeamBreaks() {}

  public DigitalInput BB1 = new DigitalInput(Constants.JunkDrawer.Beam_Break_1);
  private DigitalInput BB2 = new DigitalInput(Constants.JunkDrawer.Beam_Break_2);
  boolean beamBreak = false;

  //switch case to get different beam break sensors on the DIO
  public boolean getBeamBreak(int BeamBreakNumber) {
    switch (BeamBreakNumber) {
      case 1:
        beamBreak = BB1.get();
        break;
      case 2:
        beamBreak = BB2.get();
        break;
      default:
        beamBreak = false;
        break;
    }
    return beamBreak;
  }

  @Override
  public void periodic() {}
}