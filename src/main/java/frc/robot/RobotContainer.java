// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climber.climberDown;
import frc.robot.commands.Climber.climberUp;
import frc.robot.commands.Elevator.elevatorDown;
import frc.robot.commands.Elevator.elevatorStop;
import frc.robot.commands.Elevator.elevatorUp;
import frc.robot.commands.Intake.intakesolenoidIN;
import frc.robot.commands.Intake.intakesolenoidOut;
import frc.robot.commands.Intake.negativeClawSpeed;
import frc.robot.commands.Intake.positiveClawSpeed;
import frc.robot.commands.resetPigeon;
import frc.robot.commands.Intake.stopClawSpeed;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.subsystem.climber;
import frc.robot.subsystem.intakeClaw;
import swervelib.SwerveInputStream;

//TO-DO turn needs to be inverted
public class RobotContainer {
  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final climber Climber = new climber();
  private final Elevator elevator = new Elevator();
  private final intakeClaw intakeClaw= new intakeClaw();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operaterController = new CommandXboxController(1);
  private final Command zeroGyro = new resetPigeon(driveBase);

  private final SendableChooser<Command> autoChooser;
//github test
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientatedAngularVelocity);

    NamedCommands.registerCommand("example", Commands.print("Hello World"));
    NamedCommands.registerCommand("ZeroGyro", zeroGyro);

    //Create Intake Level, Level 1, 2, 3, 4
    NamedCommands.registerCommand("Elevator Up", new elevatorUp(elevator, 30)); //encoder ticks currently dont matter. stay below 135
    NamedCommands.registerCommand("Elevator Intake Up", new elevatorUp(elevator, 35));
    NamedCommands.registerCommand("Elevator L1 up,", new elevatorUp(elevator,30));
    NamedCommands.registerCommand("Elevator L2 up",new elevatorUp(elevator,37));
    NamedCommands.registerCommand("Elevator L3 up",new elevatorUp(elevator,38));
    NamedCommands.registerCommand("Elevator L4 up ",new elevatorUp(elevator,39));
    //Create Intake Level, Level 1, 2, 3, 4
    NamedCommands.registerCommand("Elevator Down", new elevatorDown(elevator, 15)); //encoder ticks currently dont matter. stay above 15
    NamedCommands.registerCommand("Elevator Intake Down",new elevatorDown(elevator, 35));
    NamedCommands.registerCommand("Elevator L1 Down",new elevatorDown(elevator,30));
    NamedCommands.registerCommand("Elevator L2 Down",new elevatorDown(elevator,37));
    NamedCommands.registerCommand("Elevator L3 Down", new elevatorDown(elevator,38));
    NamedCommands.registerCommand("Elevator L4 Down",new elevatorDown(elevator, 39));

    //to add auto, create auto in pathplanner
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(), 
                                            () -> driverController.getLeftY() * -1,
                                            () -> driverController.getLeftX() * -1)
                                            .withControllerRotationAxis(driverController::getRightX)
                                            .deadband(OperatorConstants.Deadzone)
                                            .scaleTranslation(1)
                                            .allianceRelativeControl(true);

  SwerveInputStream limelightDrive = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            driveBase.PIDlimslightdrive(),
                                            () -> driverController.getLeftX() * -1)
                                            .withControllerRotationAxis(driveBase.PIDlimelightRotation())
                                            .deadband(OperatorConstants.limelightDeadzone)
                                            .scaleTranslation(0.85)
                                            .allianceRelativeControl(false)
                                            .robotRelative(true);


   /*SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
                                          driverController::getRightX, 
                                          driverController::getRightY)
                                         .headingWhile(true);*/

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
                                        .allianceRelativeControl(false)
                                        .robotRelative(true);

    //Command driveFieldOrientatedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
    Command driverobotOrientedCmd = driveBase.driveFieldOriented(driveRobotOriented);
    Command limelightDriveCmd = driveBase.driveFieldOriented(limelightDrive);

  private void configureBindings() {
    driverController.y().onTrue(new resetPigeon(driveBase));
    driverController.x()
    .onTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(MathUtil.interpolate(1.0, 0.1, 1));
    }))
    .onFalse(Commands.runOnce(() -> {
      driveAngularVelocity.scaleTranslation(1);
    }));

    driverController.rightBumper().whileTrue(limelightDriveCmd);
    driverController.a().onTrue(new climberUp(Climber));
    driverController.b().onTrue(new climberDown(Climber));

    operaterController.leftBumper().whileTrue(new elevatorDown(elevator, 25));
    operaterController.rightBumper().whileTrue(new elevatorUp(elevator, 95));
    operaterController.rightTrigger(0.5).whileTrue(new positiveClawSpeed(intakeClaw));
    operaterController.leftTrigger(0.5).whileTrue(new negativeClawSpeed(intakeClaw));
    operaterController.y().onTrue(new intakesolenoidOut(intakeClaw));
    operaterController.x().onTrue(new intakesolenoidIN(intakeClaw));

    operaterController.leftBumper().whileFalse(new elevatorStop(elevator));
    operaterController.rightBumper().whileFalse(new elevatorStop(elevator));
    operaterController.leftTrigger(0.49).whileFalse(new stopClawSpeed(intakeClaw));
    operaterController.rightTrigger(0.49
    ).whileFalse(new stopClawSpeed(intakeClaw));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}