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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climberDowm;
import frc.robot.commands.climberUp;
import frc.robot.commands.elevatorDown;
import frc.robot.commands.elevatorUp;
import frc.robot.commands.resetPigeon;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.subsystem.climber;
import swervelib.SwerveInputStream;

//TO-DO turn needs to be inverted
public class RobotContainer {
  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final climber Climber = new climber();
  private final Elevator elevator = new Elevator();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operaterController = new CommandXboxController(1);
  private final Command zeroGyro = new resetPigeon(driveBase);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientatedAngularVelocity);

    NamedCommands.registerCommand("example", Commands.print("Hello World"));
    NamedCommands.registerCommand("ZeroGyro", zeroGyro);

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
    driverController.b().onTrue(new climberDowm(Climber));

    operaterController.rightTrigger().whileTrue(new elevatorUp(elevator));
    operaterController.leftTrigger().whileTrue(new elevatorDown(elevator));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}