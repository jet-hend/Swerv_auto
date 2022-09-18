// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovement.Belts;
import frc.robot.utilities.BeamBreaks;

public class SendBallToShooter extends CommandBase {
  private Belts b_belts;
  private BeamBreaks b_beambreaks;
  Boolean FIN = false ;
  /** Creates a new SendBallToShooter. */
  public SendBallToShooter(Belts b_belts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.b_belts = b_belts;
    addRequirements(b_belts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    b_belts.setbelt1(1);
    b_belts.setbelt2(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (b_beambreaks.getBeamBreak(1) == false) {
      b_belts.setbelt1(1.0);
      b_belts.setbelt2(0.4);
      System.out.println("BB1 has been hit: " + b_beambreaks.getBeamBreak(1));
      if (b_beambreaks.getBeamBreak(2) == false) {
        b_belts.setbelt1(0.0);
        b_belts.setbelt2(0.0);
        System.out.println("BB2 has been hit: " + b_beambreaks.getBeamBreak(2));
        FIN = true;
      }
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
