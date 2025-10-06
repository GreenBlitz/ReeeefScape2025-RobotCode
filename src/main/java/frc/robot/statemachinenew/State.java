package frc.robot.statemachinenew;

public enum State {

    CLOSE(Task.NONE), // done
    REMOVE_ALGAE(Task.ALGAE_REMOVE), // done
    INTAKE_ALGAE(Task.INTAKE_ALGAE), // done
    HOLD_ALGAE(Task.NONE), // done
    PROCESSOR(Task.PROCESSOR),
    NET(Task.NET), // done?
    INTAKE_CORAL(Task.INTAKE_CORAL), // done
    WHILE_DRIVE(Task.REEF),
    SCORE_REEF(Task.REEF),
    OPEN_CLIMBER(Task.CLIMB),
    CLIMB(Task.CLIMB);

    public final Task task;

    State(Task task) {
        this.task = task;
    }

    public boolean taskIs(Task other) {
        return task == other;
    }

    public enum Task {
        NONE,
        REEF,
        NET,
        PROCESSOR,
        INTAKE_CORAL,
        ALGAE_REMOVE,
        INTAKE_ALGAE,
        CLIMB;

    }

}
