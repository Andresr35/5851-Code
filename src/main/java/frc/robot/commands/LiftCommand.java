/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
  private final LiftSubsystem lift;
  double outputSpeed;

  public LiftCommand(LiftSubsystem liftSubsystem, double speed) {
    lift = liftSubsystem;
    outputSpeed = speed;
    addRequirements(lift);
  }

  @Override
  public void initialize() {
    lift.setLift(outputSpeed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    lift.LiftStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
