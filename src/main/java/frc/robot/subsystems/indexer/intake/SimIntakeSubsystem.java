package frc.robot.subsystems.indexer.intake;

import frc.robot.subsystems.indexer.constants.IndexerState;

public class SimIntakeSubsystem extends IntakeSubsystem {
    private IndexerState state;

    SimIntakeSubsystem() {
        state = IndexerState.IDLE;
    }

    @Override
    public void setIntakeState(IndexerState state) {
        this.state = state;
    }

    @Override
    protected double getCurrentAngle() {
        return state.getAngle();
    }
}