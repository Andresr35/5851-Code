/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristShooterSubsystem extends SubsystemBase {

  Spark wrist = new Spark(Constants.wrist);
  Encoder shootEncoder = new Encoder(2, 3, false, EncodingType.k4X);

  double speed;

  public void setOutput(double outputspeed) {
    speed = outputspeed;
    wrist.set(speed);
  }

 

  public void WristStop() {
    wrist.set(0);
  }

  public double angle;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle of the Wrist", (shootEncoder.getDistance() * (25 / 391.5)));
    shootEncoder.setMinRate(10);
    shootEncoder.setDistancePerPulse(1);
    shootEncoder.setReverseDirection(true);
    shootEncoder.setSamplesToAverage(127);
    shootEncoder.setMaxPeriod(.1);
    angle = shootEncoder.getDistance() * (25 / 391.5);
    
  }

  public double getAngle() {
    return angle;
  }

}
