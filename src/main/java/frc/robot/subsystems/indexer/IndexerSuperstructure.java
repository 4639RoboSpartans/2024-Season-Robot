package frc.robot.subsystems.indexer;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.intake.IntakeSubsystem;

import java.util.Objects;

public class IndexerSuperstructure extends SubsystemBase {
    private static IndexerSuperstructure instance;

    public static IndexerSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, IndexerSuperstructure::new);
    }

    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;
    private IndexerState state;

    private IndexerSuperstructure() {
        hopper = HopperSubsystem.getInstance();
        intake = IntakeSubsystem.getInstance();
        setIndexerState(IndexerState.IDLE);
    }

    private void setIndexerState(IndexerState state) {
        this.state = state;
        hopper.setIndexerState(state);
        intake.setIntakeState(state);
    }

    public Command setIndexerStateCommand(IndexerState state) {
        return Commands.runOnce(() -> setIndexerState(state));
    }

    @Override
    public void periodic() {
        if (state == IndexerState.AMP_PREP && !hopper.hasNoteTrigger().debounce(1).getAsBoolean()) {
            setIndexerState(IndexerState.AMP_READY);
        }
    }

    public Trigger hopperHasNote() {
        return hopper.hasNoteTrigger();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Indexer");
        builder.addBooleanProperty(
                "Has Note",
                this.hopperHasNote(),
                null
        );
        builder.addStringProperty(
                "State",
                () -> state.name(),
                null
        );
    }
}
