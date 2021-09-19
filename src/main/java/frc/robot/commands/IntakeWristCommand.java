/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeWristSubsystem;

public class IntakeWristCommand extends CommandBase {
  private final IntakeWristSubsystem intakeWrist;
  double outputSpeed;

  public IntakeWristCommand(IntakeWristSubsystem intakeWristSubsystem,double speed) {
    intakeWrist = intakeWristSubsystem;
    outputSpeed = speed;
    addRequirements(intakeWrist);
  }

  @Override
  public void initialize() {
    intakeWrist.setWristSpeed(outputSpeed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intakeWrist.WristStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
