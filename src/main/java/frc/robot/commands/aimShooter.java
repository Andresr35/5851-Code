/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristShooterSubsystem;

public class aimShooter extends CommandBase {
  private final WristShooterSubsystem Wrist;
  double dt, errorRate, error, outputSpeed, desiredAngle, actualAngle, lastTimeStamp, errorSum, lastError;

  public aimShooter(WristShooterSubsystem wristShooterSubsystem, double angle) {
    desiredAngle = angle;
    Wrist = wristShooterSubsystem;
    addRequirements(Wrist);
  }

  @Override
  public void initialize() {
    lastError = 0;
    errorSum = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    dt = Timer.getFPGATimestamp() - lastTimeStamp;
    actualAngle = Wrist.getAngle();
    error = actualAngle - desiredAngle;
    if (Math.abs(error) < 5.1) {
      errorSum += (error) * dt;
    }
    errorRate = (lastError - error) / dt;
    outputSpeed = error * Constants.kPWrist + Constants.kIWrist * errorSum + Constants.kDWrist * errorRate;
    Wrist.setOutput(outputSpeed);
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  @Override
  public void end(boolean interrupted) {
    Wrist.setOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
