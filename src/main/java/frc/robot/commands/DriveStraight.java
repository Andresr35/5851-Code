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

public class DriveStraight extends CommandBase {
  private final DriveSubsystem drive;


  double errorSum = 0;
  double lastTimeStamp = 0;
  double lastError = 0;
  double desiredDisplacement;

  public DriveStraight(DriveSubsystem driveSubsystem, double distance) {
    desiredDisplacement = distance;
    drive = driveSubsystem;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    errorSum = 0;
    lastError = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
    drive.configureEncoders();
   
  }

  @Override
  public void execute() {
    // get how far in feet we have gone
    double sensorPosition = drive.getRightPositioninFeet();

    // find out how much we need to go
    double error = sensorPosition - desiredDisplacement;

    // how much time has passed by
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    /*
     * if the robot is near the end, but kP is too low to provide enough voltage to
     * the motor, keep addind the error( which is small) to the time( so that
     * voltage will increase over time, until it reaches the target)
     */
    if (Math.abs(error) < Constants.driveILimit) {
      errorSum += error * dt;
    }

    // find out how fast the error is going down
    double errorRate = (error - lastError) / dt;

    // make the output speed
    double outputSpeed = Constants.kPDrive * error + Constants.kIDrive * errorSum + Constants.kDDrive * errorRate;

    drive.tankDrive(outputSpeed, outputSpeed);

    lastError = error;
    lastTimeStamp = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("Encoderposition", drive.getRightPositioninFeet());
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
