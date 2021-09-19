/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class Roller extends CommandBase {
  private final RollerSubsystem roll;
  double directionOfRoller;

  public Roller(RollerSubsystem rollerSubsystem, double direction) {
    roll = rollerSubsystem;
    directionOfRoller = direction;
    addRequirements(roll);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    roll.setRoller(directionOfRoller);
  }

  @Override
  public void end(boolean interrupted) {
    roll.rollStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
