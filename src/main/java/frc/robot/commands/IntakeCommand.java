/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsytem;

public class IntakeCommand extends CommandBase {
  private final IntakeSubsytem intake;
  double outputSpeed;

  public IntakeCommand(IntakeSubsytem intakeSubsytem, double speed) {
    intake = intakeSubsytem;
    outputSpeed = speed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeSpeed(outputSpeed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intake.IntakeStop();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
