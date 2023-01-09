package frc.robot.ShamLib.SMF.wrappers.NTComms;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;

import java.util.Set;

public class DummyNT implements INetworkTableCommunicator {
    @Override
    public void putData(String key, Sendable data) {

    }

    @Override
    public void putData(Sendable data) {

    }

    @Override
    public Sendable getData(String key) {
        return null;
    }

    @Override
    public NetworkTableEntry getEntry(String key) {
        return null;
    }

    @Override
    public boolean containsKey(String key) {
        return false;
    }

    @Override
    public Set<String> getKeys(int types) {
        return null;
    }

    @Override
    public Set<String> getKeys() {
        return null;
    }

    @Override
    public void setPersistent(String key) {

    }

    @Override
    public void clearPersistent(String key) {

    }

    @Override
    public boolean isPersistent(String key) {
        return false;
    }

    @Override
    public boolean putBoolean(String key, boolean data) {
        return false;
    }

    @Override
    public boolean setDefaultBoolean(String key, boolean defaultData) {
        return false;
    }

    @Override
    public boolean getBoolean(String key, boolean defaultValue) {
        return false;
    }

    @Override
    public boolean putNumber(String key, double data) {
        return false;
    }

    @Override
    public boolean setDefaultNumber(String key, double defaultData) {
        return false;
    }

    @Override
    public double getNumber(String key, double defaultData) {
        return 0;
    }

    @Override
    public boolean putString(String key, String data) {
        return false;
    }

    @Override
    public boolean setDefaultString(String key, String data) {
        return false;
    }

    @Override
    public String getString(String key, String defaultData) {
        return null;
    }

    @Override
    public boolean putBooleanArray(String key, boolean[] data) {
        return false;
    }

    @Override
    public boolean putBooleanArray(String key, Boolean[] data) {
        return false;
    }

    @Override
    public boolean setDefaultBooleanArray(String key, boolean[] defaultData) {
        return false;
    }

    @Override
    public boolean setDefaultBooleanArray(String key, Boolean[] defaultData) {
        return false;
    }

    @Override
    public boolean[] getBooleanArray(String key, boolean[] defaultData) {
        return new boolean[0];
    }

    @Override
    public Boolean[] getBooleanArray(String key, Boolean[] defaultData) {
        return new Boolean[0];
    }

    @Override
    public boolean putNumberArray(String key, double[] data) {
        return false;
    }

    @Override
    public boolean putNumberArray(String key, Double[] data) {
        return false;
    }

    @Override
    public boolean setDefaultNumberArray(String key, double[] defaultData) {
        return false;
    }

    @Override
    public boolean setDefaultNumberArray(String key, Double[] defaultData) {
        return false;
    }

    @Override
    public double[] getNumberArray(String key, double[] defaultData) {
        return new double[0];
    }

    @Override
    public Double[] getNumberArray(String key, Double[] defaultData) {
        return new Double[0];
    }

    @Override
    public boolean putStringArray(String key, String[] data) {
        return false;
    }

    @Override
    public boolean setDefaultStringArray(String key, String[] defaultData) {
        return false;
    }

    @Override
    public String[] getStringArray(String key, String[] defaultData) {
        return new String[0];
    }

    @Override
    public boolean putRaw(String key, byte[] data) {
        return false;
    }

    @Override
    public boolean setDefaultRaw(String key, byte[] defaultData) {
        return false;
    }

    @Override
    public byte[] getRaw(String key, byte[] defaultValue) {
        return new byte[0];
    }

    @Override
    public void postListenerTask(Runnable task) {

    }

    @Override
    public void updateValues() {

    }
}
