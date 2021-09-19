/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*___________________________________________________Imports____________________________________________*/
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveFlip;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWristCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.LimeAlignmentCommand;
import frc.robot.commands.Shooter;
import frc.robot.commands.aimShooter;
import frc.robot.commands.shooterWrist;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristShooterSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.commands.Roller;
public class RobotContainer {
  /* ____________________________Subsystems_________________________________ */
  private final Joystick Ramon = new Joystick(Constants.Ramon);
  private final Joystick Angel = new Joystick(Constants.Angel);

  private final DriveSubsystem drive = new DriveSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsytem intake = new IntakeSubsytem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
  private final LiftSubsystem lift = new LiftSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final WristShooterSubsystem wristShooter = new WristShooterSubsystem();
  private final RollerSubsystem roll = new RollerSubsystem();
  SendableChooser<Command> chooser = new SendableChooser<Command>();

  public RobotContainer() {
    
    drive.setDefaultCommand(new RunCommand(
        () -> drive.tankDrive(Constants.driveSpeed * Ramon.getRawAxis(1), Constants.driveSpeed * Ramon.getRawAxis(5)),
        drive));
    // Configure the button bindings
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    /*--------------Ramon---------------*/

    /* ____________________________Drive__________________________ */
    JoystickButton driveChange = new JoystickButton(Ramon, Constants.driveChange);
    driveChange.whenPressed(new DriveFlip(drive));

    JoystickButton driveAndAimClose = new JoystickButton(Ramon, Constants.driveAndAimClose);
    driveAndAimClose.whenPressed(new aimShooter(wristShooter,Constants.aimWristClose));
    driveAndAimClose.whileHeld(new LimeAlignmentCommand(drive,Constants.driveClose));

    JoystickButton driveAndAimFar = new JoystickButton(Ramon, Constants.driveAndAimFar);
    driveAndAimFar.whenPressed(new aimShooter(wristShooter,Constants.aimWristFar));
    driveAndAimFar.whileHeld(new LimeAlignmentCommand(drive,Constants.driveFar));

    /* _________________________Lift__________________________ */
    JoystickButton liftUp = new JoystickButton(Ramon, Constants.liftUp);
    JoystickButton liftDown = new JoystickButton(Ramon, Constants.liftDown);
    liftUp.whileHeld(new LiftCommand(lift, Constants.liftSpeed));
    liftDown.whileHeld(new LiftCommand(lift, -1 * Constants.liftSpeed));

    /* ______________________WristForShooter_____________________ */
    JoystickButton shooterUp = new JoystickButton(Ramon, Constants.shooterUp);
    JoystickButton shooterDown = new JoystickButton(Ramon, Constants.shooterDown);
    shooterDown.whileHeld(new shooterWrist(wristShooter, -1 * Constants.ShooterWristSpeed));
    shooterUp.whileHeld(new shooterWrist(wristShooter, Constants.ShooterWristSpeed));

    JoystickButton zeroShooter = new JoystickButton(Ramon, Constants.wristZero);
    zeroShooter.whenPressed(new aimShooter(wristShooter,Constants.aimWristZero));

    /* ---------Angel--------- */

    /* _______________________IntakeWrist__________________________ */
    JoystickButton IWristDown = new JoystickButton(Angel, Constants.intakeDown);
    JoystickButton IWristUp = new JoystickButton(Angel, Constants.intakeUp);
    IWristUp.whileHeld(new IntakeWristCommand(intakeWrist, Constants.intakeWristSpeed));
    IWristDown.whileHeld(new IntakeWristCommand(intakeWrist, -1 * Constants.intakeWristSpeed));

    /* _____________________Indexer__________________________ */
    JoystickButton indexIn = new JoystickButton(Angel, Constants.indexIn);
    JoystickButton indexOut = new JoystickButton(Angel, Constants.indexOut);
    indexIn.whileHeld(new IndexerCommand(indexerSubsystem, Constants.indexSpeed), false);
    indexOut.whileHeld(new IndexerCommand(indexerSubsystem, -1 * Constants.indexSpeed), false);

    /* ___________________Shooter__________________________ */
    JoystickButton Shoot = new JoystickButton(Angel, Constants.Shoot);
    Shoot.whileHeld(new Shooter(shooter));

    /* _____________________Intake__________________________ */
    JoystickButton IntakeIn = new JoystickButton(Angel, Constants.IntakeIn);
    JoystickButton IntakeOut = new JoystickButton(Angel, Constants.IntakeOut);
    IntakeIn.whileHeld(new IntakeCommand(intake, Constants.intakeSpeed));
    IntakeOut.whileHeld(new IntakeCommand(intake, -1 * Constants.intakeSpeed));

    /* _____________________Roller__________________________ */
    JoystickButton RollRight = new JoystickButton(Angel, Constants.rollright);
    JoystickButton RollLeft = new JoystickButton(Angel, Constants.rollleft);
    RollRight.whileHeld(new Roller(roll, -1 * Constants.rollerSpeed));
    RollLeft.whileHeld(new Roller(roll, Constants.rollerSpeed));

  }

  public Command getAutonomousCommand() {

    // relative to powerport with intake side being the front
    ParallelCommandGroup backUp = new ParallelCommandGroup(new DriveStraight(drive, -2));
    ParallelCommandGroup forward = new ParallelCommandGroup(new DriveStraight(drive, 2));

    SequentialCommandGroup indexing = new SequentialCommandGroup(new WaitCommand(1.5),
        new IndexerCommand(indexerSubsystem, -1 * Constants.indexSpeed).withTimeout(.3), new WaitCommand(1.5),
        new IndexerCommand(indexerSubsystem, -1 * Constants.indexSpeed).withTimeout(.5), new WaitCommand(1.5),
        new IndexerCommand(indexerSubsystem, -1 * Constants.indexSpeed).withTimeout(.5));

    ParallelCommandGroup encoderShoter = new ParallelCommandGroup(new Shooter(shooter).withTimeout(8),
        new SequentialCommandGroup(new WaitCommand(8), backUp));

    ParallelCommandGroup straightCommandGroup = new ParallelCommandGroup(new LimeAlignmentCommand(drive,Constants.driveClose).withTimeout(2),
        new aimShooter(wristShooter,Constants.aimWristClose).withTimeout(3), new Shooter(shooter), indexing);

    SequentialCommandGroup limeShot = new SequentialCommandGroup(straightCommandGroup, forward);

    return limeShot;

  }

}
