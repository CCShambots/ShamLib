package frc.robot.ShamLib.SMF.testing.state_machine;

import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

public class StateMachineTesting {
    private static ExampleStateMachine stateMachine;

    @BeforeAll
    public static void setUp() {
        stateMachine = new ExampleStateMachine("x", ExampleState.A, ExampleState.class);
    }

    @AfterAll
    public static void tearDown() {
    }

    @Test
    public void determine() {
        stateMachine.determineState();
        assert stateMachine.isDetermined();
        assert stateMachine.getState() == ExampleState.B;
    }

    private void stepSubsystem() {
        stateMachine.periodic();
    }
}

class ExampleStateMachine extends StateMachine<ExampleState> {

    public ExampleStateMachine(String name, ExampleState undeterminedState, Class<ExampleState> enumType) {
        super(name, undeterminedState, enumType);
    }

    private void declareTransitions() {
        addTransition(ExampleState.B, ExampleState.D, new InstantCommand());
        addTransition(ExampleState.D, ExampleState.C, new InstantCommand());
        addTransition(ExampleState.C, ExampleState.B, new InstantCommand());
    }

    @Override
    protected void update() {

    }

    @Override
    protected void determineSelf() {
        setState(ExampleState.B);
    }
}

enum ExampleState {
    A, B, C, D, E
}