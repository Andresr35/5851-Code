/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWristSubsystem extends SubsystemBase {

  WPI_TalonSRX wrist = new WPI_TalonSRX(Constants.intakeWrist);
  PigeonIMU gyro = new PigeonIMU(wrist);

  double[] yprGyro = new double[3];
  PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();

  public IntakeWristSubsystem() {
    wrist.setNeutralMode(NeutralMode.Brake);

  }

  public void setWristSpeed(double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }
  public void WristStop() {
    wrist.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    gyro.getGeneralStatus(genStatus);
    gyro.getYawPitchRoll(yprGyro);
    SmartDashboard.putNumber("Gyro Heading", yprGyro[0]);
    SmartDashboard.putNumber("Gyro Pitch", yprGyro[1]);
    SmartDashboard.putNumber("Gyro Roll", yprGyro[2]);

  }
}
