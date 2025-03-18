// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intakeClaw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class negativeClawSpeedAuto extends Command {
  intakeClaw intakeClaw;
  Timer autoTimer = new Timer();
  /** Creates a new negativeClawSpeed. */
  public negativeClawSpeedAuto(intakeClaw intakeClaw) {
    this.intakeClaw = intakeClaw;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.reset();
    System.out.println("SlayQueen");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoTimer.start();
    intakeClaw.clawSpeed(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(autoTimer.get() >= 3.0){
      autoTimer.stop();
      return true;
    } else{
      return false;
    }
  }
}
