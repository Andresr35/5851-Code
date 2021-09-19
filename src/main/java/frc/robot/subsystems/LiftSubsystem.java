/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  WPI_VictorSPX liftOne = new WPI_VictorSPX(Constants.liftOne);
  WPI_VictorSPX liftTwo = new WPI_VictorSPX(Constants.liftTwo);

  public LiftSubsystem() {
    liftOne.setNeutralMode(NeutralMode.Brake);
    liftTwo.setNeutralMode(NeutralMode.Brake);
  }

  public void setLift(double speed) {
    liftOne.set(ControlMode.PercentOutput, speed);
    liftTwo.set(ControlMode.PercentOutput, speed);
  }

  

  public void LiftStop() {
    liftOne.set(ControlMode.PercentOutput, 0);
    liftTwo.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
  }

}
