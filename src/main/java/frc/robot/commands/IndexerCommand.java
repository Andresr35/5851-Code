/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  double outputSpeed;
  public IndexerCommand(IndexerSubsystem indexerSubsystem,double speed) {
    outputSpeed= speed;
    indexer = indexerSubsystem;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    indexer.setIndexSpeed(outputSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.IndexStop();
    
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
