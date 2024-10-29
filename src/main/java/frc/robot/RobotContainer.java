// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Controls.*;
import frc.robot.constants.InterpolatingTables;
import frc.robot.oi.OI;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    public static OI oi;
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final ShooterSuperstructure shooter;
    private final IndexerSuperstructure indexer;
    private final SendableChooser<Command> autos;


    public RobotContainer() {
        InterpolatingTables.initializeTables();

        swerveDriveSubsystem = CommandSwerveDrivetrain.getInstance();
        shooter = ShooterSuperstructure.getInstance();
        indexer = IndexerSuperstructure.getInstance();

        autos = new SendableChooser<>();
        autos.setDefaultOption("null auto", new WaitCommand(1));

        SmartDashboard.putData("Autons", autos);

        configureBindings();
    }

    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(
                swerveDriveSubsystem.driveFieldCentricCommand()
        );

        DriverControls.SOTF.and(indexer.hopperHasNote())
                .whileTrue(
                        Commands.parallel(
                                swerveDriveSubsystem.SOTFCommand(),
                                Commands.sequence(
                                        Commands.waitUntil(
                                                swerveDriveSubsystem.inShootingRange()
                                        ),
                                        Commands.parallel(
                                                Commands.waitUntil(
                                                        swerveDriveSubsystem.isAligned()
                                                                .and(swerveDriveSubsystem.inShootingSector())
                                                                .and(swerveDriveSubsystem.inShootingRange())
                                                ),
                                                shooter.setShootingMode(ShootingMode.SPINUP)
                                        ),
                                        CommandFactory.autoShootCommand()
                                )
                        )
                ).onFalse(
                        CommandFactory.resetCommand()
                );

        DriverControls.AimButton
                .whileTrue(
                        Commands.sequence(
                                swerveDriveSubsystem.pathfindCommand(
                                        AimUtil.getManualSpeakerPose()
                                ),
                                CommandFactory.autoShootCommand()
                        )
                ).onFalse(
                        CommandFactory.resetCommand()
                );

        OperatorControls.IntakeButton
                .whileTrue(
                        indexer.setIndexerStateCommand(IndexerState.INTAKING)
                )
                .onFalse(
                        indexer.setIndexerStateCommand(IndexerState.EXTENDED)
                );

        OperatorControls.IntakeRetractButton
                .onTrue(
                        indexer.setIndexerStateCommand(IndexerState.IDLE)
                );

        OperatorControls.IntakeExtendButton
                .onTrue(
                        indexer.setIndexerStateCommand(IndexerState.EXTENDED)
                );

        OperatorControls.OuttakeButton
                .whileTrue(
                        indexer.setIndexerStateCommand(IndexerState.OUTTAKING)
                )
                .onFalse(
                        indexer.setIndexerStateCommand(IndexerState.IDLE)
                );

        OperatorControls.RunSpeakerShooterButton.and(indexer.hopperHasNote())
                .whileTrue(
                        CommandFactory.autoShootCommand()
                ).onFalse(
                        CommandFactory.resetCommand()
                );

        OperatorControls.ManualShooterButton
                .whileTrue(
                        Commands.parallel(
                                shooter.setShootingMode(ShootingMode.MANUAL),
                                indexer.setIndexerStateCommand(IndexerState.FEED)
                        )
                ).onFalse(
                        CommandFactory.resetCommand()
                );
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }

    public void sendSubsystems() {
        SmartDashboard.putData(swerveDriveSubsystem);
        SmartDashboard.putData(indexer);
    }
}