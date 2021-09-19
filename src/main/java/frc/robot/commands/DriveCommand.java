/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final double leftSpeed, rightSpeed;

  public DriveCommand(DriveSubsystem driveSubsystem, double leftDriveSpeed, double rightDriveSpeed) {
    drive = driveSubsystem;
    leftSpeed = leftDriveSpeed;
    rightSpeed = rightDriveSpeed;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.configureEncoders();
  }

  @Override
  public void execute() {

    drive.tankDrive(Constants.driveSpeed * leftSpeed, Constants.driveSpeed * rightSpeed);
    System.out.println("teleop is using the tankDrive");
   
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
