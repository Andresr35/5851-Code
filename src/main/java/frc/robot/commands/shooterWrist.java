/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristShooterSubsystem;

public class shooterWrist extends CommandBase {
  private final WristShooterSubsystem wristShooter;
  double outputSpeed;

  public shooterWrist(WristShooterSubsystem wristShooterSubsystem,double speed) {
    outputSpeed = speed;
    wristShooter = wristShooterSubsystem;
    addRequirements(wristShooter);
  }

  @Override
  public void initialize() {
    wristShooter.setOutput(outputSpeed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    wristShooter.WristStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
