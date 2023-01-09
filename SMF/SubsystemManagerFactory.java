package frc.robot.ShamLib.SMF;

import frc.robot.ShamLib.SMF.wrappers.NTComms.INetworkTableCommunicator;

public class SubsystemManagerFactory {
    private static SubsystemManager instance;

    private static INetworkTableCommunicator defaultNTComms;

    public static SubsystemManager getInstance() {
        if (instance == null) {
            instance = new SubsystemManager(defaultNTComms);
        }

        return instance;
    }

    public static void reset() {
        instance = new SubsystemManager(defaultNTComms);
    }

    public static void setInstance(SubsystemManager manager) {
        instance = manager;
    }

    public static void setDefaultNTComms(INetworkTableCommunicator comms) {
        defaultNTComms = comms;
    }

    public static INetworkTableCommunicator getDefaultNTComms() {
        return defaultNTComms;
    }
}
