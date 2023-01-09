package frc.robot.ShamLib.SMF.wrappers.NTComms;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;

import java.util.Set;

public interface INetworkTableCommunicator {
    void putData(String key, Sendable data);
    void putData(Sendable data);

    Sendable getData(String key);
    NetworkTableEntry getEntry(String key);

    boolean containsKey(String key);

    Set<String> getKeys(int types);
    Set<String> getKeys();

    void setPersistent(String key);
    void clearPersistent(String key);
    boolean isPersistent(String key);

    boolean putBoolean(String key, boolean data);
    boolean setDefaultBoolean(String key, boolean defaultData);
    boolean getBoolean(String key, boolean defaultValue);

    boolean putNumber(String key, double data);
    boolean setDefaultNumber(String key, double defaultData);
    double getNumber(String key, double defaultData);

    boolean putString(String key, String data);
    boolean setDefaultString(String key, String data);
    String getString(String key, String defaultData);

    boolean putBooleanArray(String key, boolean[] data);
    boolean putBooleanArray(String key, Boolean[] data);
    boolean setDefaultBooleanArray(String key, boolean[] defaultData);
    boolean setDefaultBooleanArray(String key, Boolean[] defaultData);
    boolean[] getBooleanArray(String key, boolean[] defaultData);
    Boolean[] getBooleanArray(String key, Boolean[] defaultData);

    boolean putNumberArray(String key, double[] data);
    boolean putNumberArray(String key, Double[] data);
    boolean setDefaultNumberArray(String key, double[] defaultData);
    boolean setDefaultNumberArray(String key, Double[] defaultData);
    double[] getNumberArray(String key, double[] defaultData);
    Double[] getNumberArray(String key, Double[] defaultData);

    boolean putStringArray(String key, String[] data);
    boolean setDefaultStringArray(String key, String[] defaultData);
    String[] getStringArray(String key, String[] defaultData);

    boolean putRaw(String key, byte[] data);
    boolean setDefaultRaw(String key, byte[] defaultData);
    byte[] getRaw(String key, byte[] defaultValue);

    void postListenerTask(Runnable task);

    void updateValues();
}
