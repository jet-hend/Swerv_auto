// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovement.Intake;
import frc.robot.utilities.AirSystem;


public class IntakeControl extends CommandBase {
  private Intake i_intake; 
  private AirSystem a_airsystem;
  private double PO;
  private boolean state;
  private Boolean FIN = false;
  /** Creates a new IntakeControl. */
  public IntakeControl(Intake i_intake, double PO, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.i_intake = i_intake;
    this.PO = PO;
    this.state = state;
    addRequirements(i_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == false) {
      i_intake.setIntakeMotor(0);
      FIN = true;
    } else {
      i_intake.setIntakeMotor(PO);
      a_airsystem.setIntakeSolenoid(state);;
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
