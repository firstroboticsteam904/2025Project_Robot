// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorControl extends Command {
  /** Creates a new elevatorControl. */
  PIDController elevatorController = new PIDController(1, 0, 0);
  Elevator kElevator;
  double desiredElevatorTicks;

  public elevatorControl(Elevator kElevator, int elevatorTicks) {
    this.kElevator = kElevator;
    desiredElevatorTicks = elevatorTicks;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kElevator.elevatorSpeed(elevatorController.calculate(kElevator.ElevatorTravel(), desiredElevatorTicks));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(kElevator.ElevatorTravel() >= 127){
      kElevator.elevatorSpeed(0.0);
      interrupted = true;
    } else{
      interrupted = false;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevatorController.atSetpoint()){
      kElevator.elevatorSpeed(-0.025);
      return true;
    } else {
      return false;
    }
  }
}
