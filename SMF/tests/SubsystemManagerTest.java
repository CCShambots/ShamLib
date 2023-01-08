package frc.robot.ShamLib.SMF.tests;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.SMF.StatedSubsystem;
import frc.robot.ShamLib.SMF.SubsystemManager;

import java.util.ArrayList;
import java.util.List;

class SubsystemManagerTest {
    SubsystemManager manager = SubsystemManager.getInstance();

    @org.junit.jupiter.api.BeforeEach
    void setUp() {
        manager.reset();
    }

    @org.junit.jupiter.api.AfterEach
    void tearDown() {

    }

    //TODO: want to add another one of these that registers 2 of the same to check if throws error like it should
    @org.junit.jupiter.api.Test
    void registerSubsystem() {
        //TODO: add diagnostics methods (already in the teamhood thing)

        ExampleSubsystem1 sub = new ExampleSubsystem1();

        manager.registerSubsystem(sub);

        assert manager.getSubsystems().contains(sub);
    }

    @org.junit.jupiter.api.Test
    void sendSubsystemToNT() {
        //TODO: this will prob not work cause it uses dashboard which will probs not work without sim
        //this is why we should do more dependency injection
        //basically a wrapper for data output and we could make it more tailored to our usecase (possibly logging things or whatever)
        //dependency injection is annoying with singletons/statics tho so i wonder if we could avoid making subsystemmanager a singleton?

        ExampleSubsystem1 sub = new ExampleSubsystem1();

        manager.sendSubsystemToNT(sub);

        assert SmartDashboard.getData(sub.getName()) != null;
    }

    //TODO: want to add another one of these that registers 2 of the same to check if throws error like it should
    @org.junit.jupiter.api.Test
    void registerSubsystems() {
        ExampleSubsystem1 sub1 = new ExampleSubsystem1();
        ExampleSubsystem2 sub2 = new ExampleSubsystem2();

        //...
        //!
        manager.registerSubsystems(sub1, sub2);

        assert manager.getSubsystems().contains(sub1);
        assert manager.getSubsystems().contains(sub2);
    }

    @org.junit.jupiter.api.Test
    void determineAllSubsystems() {
        manager.registerSubsystems(new ExampleSubsystem1(), new ExampleSubsystem2());

        Command c = manager.determineAllSubsystems();

        /* simulating the command scheduler running this command cause this test wouldnt
        * see the error if i just sent it to the scheduler cause it would run it in a seperate block
        * */
        c.initialize();
        c.execute();
        c.end(false);
    }

    @org.junit.jupiter.api.Test
    void enableAllSubsystems() {
        manager.registerSubsystems(new ExampleSubsystem1(), new ExampleSubsystem2());

        /* would surely throw error if something went really wrong
        * dont want to loop through registered subsystems to see if they are enabled because
        * it would be a subsystem issue if it didnt become enabled after enable() was invoked
        *
        * but idk???
        * (I'm not very smart)
        * */
        manager.enableAllSubsystems();
    }

    @org.junit.jupiter.api.Test
    void disableAllSubsystems() {
        manager.registerSubsystems(new ExampleSubsystem1(), new ExampleSubsystem2());

        //same deal as enableAllSubsystems test, idk if this should even be a test
        //should i be enabling them all first?

        manager.disableAllSubsystems();
    }

    @org.junit.jupiter.api.Test
    void prepSubsystems() {
        manager.registerSubsystems(new ExampleSubsystem1(), new ExampleSubsystem2());

        //this is just a combo of two other tests (enable all and determine all), no?
        manager.prepSubsystems();
    }
}

class ExampleSubsystem2 extends StatedSubsystem<ExampleSubsystem2.ExampleState2> {

    public ExampleSubsystem2() {
        super(ExampleState2.class);
    }

    @Override
    public void update() {

    }

    @Override
    public String getName() {
        return "example_subsystem_1";
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

    }

    enum ExampleState2 {
        STATE_1,
        STATE_2,
        STATE_3
    }
}

class ExampleSubsystem1 extends StatedSubsystem<ExampleSubsystem1.ExampleState1> {

    public ExampleSubsystem1() {
        super(ExampleState1.class);
    }

    @Override
    public void update() {

    }

    @Override
    public String getName() {
        return "example_subsystem_1";
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

    }

    enum ExampleState1 {
        STATE_1,
        STATE_2,
        STATE_3
    }
}