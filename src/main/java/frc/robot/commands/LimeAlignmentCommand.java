/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class LimeAlignmentCommand extends CommandBase {
  private final DriveSubsystem drive;
  double currentDistance, lastError = 0, errorRate, error, desiredDistance, speed, errorSum, dt, lastTimeStamp, angle,
      errorSumSteer, lastErrorSteer, errorRateSteer, ksteering, leftSpeed, rightSpeed;

  public LimeAlignmentCommand(DriveSubsystem driveSubsystem, double distance) {
    desiredDistance = distance;
    drive = driveSubsystem;
    addRequirements(drive);
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
    angle = drive.x;
    if (Math.abs(angle) < Constants.steerILimit) {
      errorSumSteer += (-angle) * dt;
    }
    errorRateSteer = (lastErrorSteer - angle) / dt;

    ksteering = Constants.kPSteer * -angle + Constants.kISteer * errorSumSteer + Constants.kDSteer * errorRateSteer;

    lastErrorSteer = angle;

    currentDistance = drive.getEstimatedDistance();
    error = currentDistance - desiredDistance;

    if (Math.abs(error) < Constants.driveILimit) {
      errorSum += (error) * dt;
    }

    errorRate = (error - lastError) / dt;
    speed = Constants.kPDrive * error + Constants.kIDrive * errorSum + Constants.kDDrive * errorRate;
    leftSpeed = speed + ksteering;
    rightSpeed = speed - ksteering;
    drive.tankDrive(leftSpeed, rightSpeed);
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;
    SmartDashboard.putNumber("Distance Off Target", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
