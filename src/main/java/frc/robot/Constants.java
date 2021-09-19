// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
// this is the constants class
// this is the second change
//this is a test for branch


public final class Constants {

        public static int

        Ramon = 0, Angel = 1,
                        // Ramon's Buttons
                        driveAndAimClose = 5, liftUp = 6, driveAndAimFar = 7, liftDown = 8, shooterUp = 9,
                        shooterDown = 10, driveChange = 12, driveAim = 11, wristZero = 13,

                        // Angel's Buttons
                        rollright = 1, rollleft = 2, indexOut = 3, indexIn = 4, Shoot = 5, IntakeIn = 6, IntakeOut = 8,
                        intakeDown = 12, intakeUp = 11,

                        // DeviceNumbers

                        /*------------------------Victor SPX Motor Contollers--------------------*/
                        rr = 11, liftOne = 14, liftTwo = 15, roller = 16, indexer = 17,

                        /*------------------------Talon SRX Motor Controllers--------------------*/
                        lf = 12, intakeWrist = 13, rf = 18,

                        /*------------------------Spark Motor Controllers------------------------*/
                        lr = 1, wrist = 2, shooterOne = 19, shooterTwo = 4,

                        /*-------------------------Victor Motor Controllers-----------------------*/
                        intake = 5;

        // Speeds
        public static double
        /* Drive Speed */
        driveSpeed = 0.95,

                        // Lift and Rollers
                        liftSpeed = 1, rollerSpeed = 0.5,

                        // IntakeWrist, Intake, Indexer
                        intakeWristSpeed = 1, indexSpeed = 0.6, intakeSpeed = 0.5,

                        // Shooter and ShooterWrist
                        shooterSpeed = 0.8, ShooterWristSpeed = 0.5,

                        // Drive PID
                        // -0.2, -.25, 0.0015 1.5 manual
                        kPDrive = -.2, kIDrive = -0.25, kDDrive = -0.0015, driveILimit = 1.5,

                        // DriveAutoPID
                        kPDriveAuto = -0.4, kIDriveAuto = -0.1, kDDriveAuto = -0.0015, driveILimitAuto = 2,

                        // SteerAutoPID
                        kPSteerAuto = 0.2, kISteerAuto = 0.3, kDSteerAuto = 0.01, steerILimitAuto = 7,

                        // Steer PID
                        // 0.1 0.45 0.01 7
                        kPSteer = 0.1, kISteer = 0.45, kDSteer = 0.01, steerILimit = 7,

                        // Wrist PID
                        kPWrist = 0.3, kIWrist = 0.11, kDWrist = 0,

                        // Wrist Angles
                        aimWristClose = -30, aimWristFar = -25, aimWristZero = 0,

                        // Drive Distances
                        driveClose = 12, driveFar = 19;

}