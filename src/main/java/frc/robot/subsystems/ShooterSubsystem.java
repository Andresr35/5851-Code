/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  WPI_TalonSRX shooterTwo = new WPI_TalonSRX(Constants.shooterOne);
  Spark shooter = new Spark(Constants.shooterTwo);

  public void Shoot() {
    shooter.set(-Constants.shooterSpeed);
    shooterTwo.set(ControlMode.PercentOutput,Constants.shooterSpeed);
  }

  public void Stop() {
    shooter.set(0);
    shooterTwo.set(0);
  }

  @Override
  public void periodic() {

  }
}
