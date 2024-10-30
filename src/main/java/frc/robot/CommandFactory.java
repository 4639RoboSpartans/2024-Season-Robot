package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class CommandFactory {
    private static final ShooterSuperstructure shooter = ShooterSuperstructure.getInstance();
    private static final IndexerSuperstructure indexer = IndexerSuperstructure.getInstance();
    private static final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static Command autoShootCommand() {
        if (Robot.isReal() || !Robot.isInAuton())
            return Commands.deadline(
                    Commands.sequence(
                            Commands.waitUntil(
                                    shooter.atRequestedStateTrigger()
                            ),
                            indexer.setIndexerStateCommand(IndexerState.FEED),
                            Commands.waitUntil(indexer.hopperHasNote().negate())
                    ),
                    Commands.repeatingSequence(
                            Commands.either(
                                    shooter.setShootingMode(ShootingMode.AUTO)
                                            .alongWith(indexer.setIndexerStateCommand(IndexerState.FEED)),
                                    shooter.setShootingMode(ShootingMode.IDLE)
                                            .alongWith(indexer.setIndexerStateCommand(IndexerState.IDLE)),
                                    swerve.inShootingRange().and(swerve.inShootingSector())
                            )
                    )
            ).andThen(
                    resetCommand()
            );
        else
            return Commands.deadline(
                    Commands.waitSeconds(2),
                    Commands.waitUntil(shooter.atRequestedStateTrigger())
                            .andThen(indexer.setIndexerStateCommand(IndexerState.FEED)),
                    shooter.setShootingMode(ShootingMode.AUTO)
            ).andThen(
                    resetCommand()
            );
    }

    public static Command resetCommand() {
        return Commands.parallel(
                indexer.setIndexerStateCommand(IndexerState.IDLE),
                shooter.setShootingMode(ShootingMode.IDLE)
        );
    }

    public static Command waitThenShootCommand() {
        return Commands.sequence(
                Commands.waitUntil(
                        swerve.inShootingRange()
                ),
                Commands.parallel(
                        Commands.waitUntil(
                                swerve.isAligned()
                                        .and(swerve.inShootingSector())
                                        .and(swerve.inShootingRange())
                        ),
                        shooter.setShootingMode(ShootingMode.SPINUP)
                ),
                CommandFactory.autoShootCommand()
        );
    }
}
