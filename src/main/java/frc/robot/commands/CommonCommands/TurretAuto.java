// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommonCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallMovement.Turret;
import frc.robot.utilities.LimeLight;

public class TurretAuto extends CommandBase {
  /** Creates a new TurretAuto. */

  private LimeLight l_limeLight;
  private Turret t_turret;
  private boolean state;
  private boolean FIN = false;
  public TurretAuto(LimeLight l_limelight, Turret t_turret, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(l_limeLight, t_turret);
    this.l_limeLight = l_limelight;
    this.t_turret = t_turret;
    this.state = state;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pipeline 1 for blue balls
    //pipeline 2 for red balls
    l_limeLight.setPipeline(0);

    System.out.println("start AutoAim");
    t_turret.setTurretPO(0.0);
    t_turret.TurretM.setSelectedSensorPosition(0);
    FIN = false;
  }
  //allowed error
  double offset = 0.5;

  //limits
  double Rlimit = 55000.0;
  double Llimit = -38873.0;

  //stage one marker
  double stage1 = 8;
  //stage two marker
  double stage2 = 6;
  //stage three marker
  double stage3 = 2;

  //turn speed 1
  double speed1 = 1.0;
  //turn speed 2
  double speed2 = 0.5;
  //turn speed 3
  double speed3 = 0.2;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimeLight.hasTarget() && state != false) {

      if (t_turret.getTurretEncoder() > Rlimit || t_turret.getTurretEncoder() < Llimit) {
        t_turret.setTurretPO(0);
      }

      //stage 1
      if (l_limeLight.getHorizontalOffset() > stage1 || l_limeLight.getHorizontalOffset() < -stage1) {
          if (l_limeLight.getHorizontalOffset() > stage1){
            t_turret.setTurretPO(speed1);
          }
          if (l_limeLight.getHorizontalOffset() < -stage1){
            t_turret.setTurretPO(-speed1);
          }
        System.out.println("stage1:" + l_limeLight.getHorizontalOffset());
        System.out.println(t_turret.TurretM.getSelectedSensorPosition());
      }
      //stage 2
      if (l_limeLight.getHorizontalOffset() > stage2 || l_limeLight.getHorizontalOffset() < -stage2) {
          if (l_limeLight.getHorizontalOffset() > stage2){
            t_turret.setTurretPO(speed2);
          }
          if (l_limeLight.getHorizontalOffset() < -stage2){
            t_turret.setTurretPO(-speed2);
          }
        System.out.println("stage2:" + l_limeLight.getHorizontalOffset());
        System.out.println(t_turret.TurretM.getSelectedSensorPosition());
      }

      //stage 3
      if (l_limeLight.getHorizontalOffset() > stage3 || l_limeLight.getHorizontalOffset() < -stage3) {
          if (l_limeLight.getHorizontalOffset() > stage3){
            t_turret.setTurretPO(speed3);
          }
          if (l_limeLight.getHorizontalOffset() < -stage3){
            t_turret.setTurretPO(-speed3);
          }
        System.out.println("stage3:" + l_limeLight.getHorizontalOffset());
        System.out.println(t_turret.TurretM.getSelectedSensorPosition());
      }

      if (l_limeLight.getHorizontalOffset() > -offset && l_limeLight.getHorizontalOffset() < offset){
        t_turret.setTurretPO(0);
        System.out.println("Zero Error: SHOOT!!s" + l_limeLight.getHorizontalOffset());
        System.out.println(t_turret.TurretM.getSelectedSensorPosition());
        RobotContainer.XboxC.setRumble(RumbleType.kLeftRumble, 0.3);
        RobotContainer.XboxC.setRumble(RumbleType.kRightRumble, 0.3);
      }

      if (t_turret.getTurretEncoder() > Rlimit || t_turret.getTurretEncoder() < Llimit) {
        t_turret.setTurretPO(0);
      }
    } else {
      t_turret.setTurretPOS(0);
      try {
        Thread.sleep(1500);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      t_turret.setTurretPO(0);
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