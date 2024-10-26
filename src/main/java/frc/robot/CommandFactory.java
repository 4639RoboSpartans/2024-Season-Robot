package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class CommandFactory {
    private static final ShooterSuperstructure shooter = ShooterSuperstructure.getInstance();
    private static final IndexerSuperstructure indexer = IndexerSuperstructure.getInstance();

    public static Command autoShootCommand() {
        return Commands.race(
                Commands.sequence(
                        Commands.waitUntil(
                                shooter.atRequestedStateTrigger()
                        ),
                        indexer.setIndexerStateCommand(IndexerState.FEED)
                                .until(indexer.hopperHasNote()),
                        Commands.parallel(
                                indexer.setIndexerStateCommand(IndexerState.IDLE),
                                shooter.setShootingMode(ShootingMode.IDLE)
                        )
                ),
                shooter.setShootingMode(ShootingMode.AUTO)
        );
    }
}
