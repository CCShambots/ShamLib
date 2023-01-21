package frc.robot.ShamLib.SMF;

public class SubsystemManagerFactory {

    private static SubsystemManager instance;

    public static SubsystemManager getInstance() {
        if(instance == null) instance = new SubsystemManager();
        return instance;
    }

    public static void setInstance(SubsystemManager instanceToSet) {
        instance = instanceToSet;
    }


}
