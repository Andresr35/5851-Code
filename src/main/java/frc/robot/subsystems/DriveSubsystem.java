/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private boolean orientation = false;

  public double leftsp, rightsp, rightDrivetoFeet, leftDrivetoFeet, x, y, area, distance, cameraHeight = 15.825,
      portHeight = 92, bottomAngle = 25, angle, inchToFeet = 12;

  private final double kDriveTicktoFeet = (6 * Math.PI) / 49152;

  WPI_TalonSRX leftFrontDrive;
  Spark leftRearDrive;
  SpeedControllerGroup leftDrive;



  WPI_VictorSPX rightFrontDrive;
  WPI_TalonSRX rightRearDrive;
  SpeedControllerGroup rightDrive;

  DifferentialDrive drive;

  public DriveSubsystem() {
    this.leftFrontDrive = new WPI_TalonSRX(Constants.lf);
    this.leftRearDrive = new Spark(Constants.lr);
    this.leftDrive = new SpeedControllerGroup(leftFrontDrive, leftRearDrive);

    // --------------------------------RightDrive-----------------------------------------\\
    this.rightFrontDrive = new WPI_VictorSPX(Constants.rf);
    this.rightRearDrive = new WPI_TalonSRX(Constants.rr);
    this.rightDrive = new SpeedControllerGroup(rightFrontDrive, rightRearDrive);

    // ---------------------------------defaultDrive------------------------------\\
    this.drive = new DifferentialDrive(leftDrive, rightDrive);
    
    rightRearDrive.setSelectedSensorPosition(0);
    leftFrontDrive.setSelectedSensorPosition(0);
    rightRearDrive.setNeutralMode(NeutralMode.Brake);
    leftFrontDrive.setNeutralMode(NeutralMode.Brake);
    rightFrontDrive.setNeutralMode(NeutralMode.Brake);

  }

  // --------------------------------flipsOrientaion-------------\\
  public void flipMotors() {
    orientation = !orientation;

    rightDrive.setInverted(orientation);
    leftDrive.setInverted(orientation);
  }

  // ------------------------DriveMethod--------------------\\
  public void tankDrive(double leftSpeed, double rightSpeed) {

    leftsp = leftSpeed;
    rightsp = rightSpeed;
    if (orientation) {
      drive.tankDrive(leftsp, rightsp);

    } else {
      drive.tankDrive(rightsp, leftsp);
    }
  }

  // -------------------------------setUpEncoders-----------------------------\\
  public void configureEncoders() {
    rightRearDrive.setSensorPhase(true);

    rightRearDrive.setSelectedSensorPosition(0);
    leftFrontDrive.setSelectedSensorPosition(0);

  }

  double[] yprGyro = new double[3];
  PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();

  @Override
  public void periodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    rightDrivetoFeet = ((rightRearDrive.getSelectedSensorPosition() * kDriveTicktoFeet));
    leftDrivetoFeet = ((leftFrontDrive.getSelectedSensorPosition() * kDriveTicktoFeet));
    SmartDashboard.putNumber("Left Distance", leftDrivetoFeet);
    SmartDashboard.putNumber("Right Distance", rightDrivetoFeet);

    SmartDashboard.putBoolean("Drive Inverted?", !rightDrive.getInverted());

    // ---------------------------------Limelight---------------------\\
    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("Lime Yaw", x);
    SmartDashboard.putNumber("Lime Pitch", y);
    SmartDashboard.putNumber("Limelight Area", area);

    SmartDashboard.putNumber("FPGAtime", Timer.getFPGATimestamp());
    distance = ((portHeight - cameraHeight) / Math.tan(Math.toRadians(bottomAngle + y)));
    SmartDashboard.putNumber("Distance to PowerPort", distance / inchToFeet);
  }

  public double getHeadingOffset() {

    return x;
  }

  public double getVerticalOffset() {

    return y;
  }

  public double getEstimatedDistance() {
    return distance / inchToFeet;
  }

  public double getleftPositioninFeet() {

    return leftDrivetoFeet;
  }

  public double getRightPositioninFeet() {
    return rightDrivetoFeet;
  }
}
