package frc.robot.autos;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommandFactory;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.constants.SwerveConstants;

public class AutoHelper {
    private static CommandSwerveDrivetrain swerveDrivetrain = CommandSwerveDrivetrain.getInstance();
    private static IndexerSuperstructure indexer = IndexerSuperstructure.getInstance();

    public static Command autoIntakeCommand() {
        return indexer.setIndexerStateCommand(IndexerState.INTAKING);
    }

    public static Command intakeWhileMoving(String pathName) {
        Command moveCommand = swerveDrivetrain.followChoreoPath(pathName, true);
        return Commands.deadline(
                moveCommand,
                Commands.sequence(
                        Commands.waitSeconds(
                                Math.max(
                                        0,
                                        Choreo.getTrajectory(pathName).getTotalTime() - SwerveConstants.TIME_BEFORE_INTAKE_START
                                )
                        ),
                        indexer.setIndexerStateCommand(IndexerState.INTAKING)
                )
        ).andThen(
                Commands.sequence(
                        Commands.waitUntil(indexer.hopperHasNote()),
                        indexer.setIndexerStateCommand(IndexerState.IDLE)
                )
        );
    }

    public static Command SOTFThenIntake(String pathName) {
        Command moveCommand = swerveDrivetrain.followChoreoPath(pathName, true);
        return Commands.deadline(
                moveCommand,
                Commands.deadline(
                        Commands.waitSeconds(
                                Math.max(
                                        0,
                                        Choreo.getTrajectory(pathName).getTotalTime() - SwerveConstants.TIME_BEFORE_INTAKE_START
                                )
                        ),
                        CommandFactory.waitThenShootCommand()
                ).andThen(
                        indexer.setIndexerStateCommand(IndexerState.INTAKING)
                )
        ).andThen(
                Commands.sequence(
                        Commands.waitUntil(indexer.hopperHasNote()),
                        indexer.setIndexerStateCommand(IndexerState.IDLE)
                )
        );
    }

    public static Command followThenShoot(String pathname) {
        return swerveDrivetrain.followChoreoPath(pathname, true)
                .andThen(CommandFactory.autoShootCommand());
    }
}