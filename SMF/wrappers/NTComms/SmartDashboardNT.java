package frc.robot.ShamLib.SMF.wrappers.NTComms;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;

public class SmartDashboardNT implements INetworkTableCommunicator {
    @Override
    public void putData(String key, Sendable data) {
        SmartDashboard.putData(key, data);
    }

    @Override
    public void putData(Sendable data) {
        SmartDashboard.putData(data);
    }

    @Override
    public Sendable getData(String key) {
        return SmartDashboard.getData(key);
    }

    @Override
    public NetworkTableEntry getEntry(String key) {
        return SmartDashboard.getEntry(key);
    }

    @Override
    public boolean containsKey(String key) {
        return SmartDashboard.containsKey(key);
    }

    @Override
    public Set<String> getKeys(int types) {
        return SmartDashboard.getKeys(types);
    }

    @Override
    public Set<String> getKeys() {
        return SmartDashboard.getKeys();
    }

    @Override
    public void setPersistent(String key) {
        SmartDashboard.setPersistent(key);
    }

    @Override
    public void clearPersistent(String key) {
        SmartDashboard.clearPersistent(key);
    }

    @Override
    public boolean isPersistent(String key) {
        return SmartDashboard.isPersistent(key);
    }

    @Override
    public boolean putBoolean(String key, boolean data) {
        return SmartDashboard.putBoolean(key, data);
    }

    @Override
    public boolean setDefaultBoolean(String key, boolean defaultData) {
        return SmartDashboard.setDefaultBoolean(key, defaultData);
    }

    @Override
    public boolean getBoolean(String key, boolean defaultValue) {
        return SmartDashboard.getBoolean(key, defaultValue);
    }

    @Override
    public boolean putNumber(String key, double data) {
        return SmartDashboard.putNumber(key, data);
    }

    @Override
    public boolean setDefaultNumber(String key, double defaultData) {
        return SmartDashboard.setDefaultNumber(key, defaultData);
    }

    @Override
    public double getNumber(String key, double defaultData) {
        return SmartDashboard.getNumber(key, defaultData);
    }

    @Override
    public boolean putString(String key, String data) {
        return SmartDashboard.putString(key, data);
    }

    @Override
    public boolean setDefaultString(String key, String data) {
        return SmartDashboard.setDefaultString(key, data);
    }

    @Override
    public String getString(String key, String defaultData) {
        return SmartDashboard.getString(key, defaultData);
    }

    @Override
    public boolean putBooleanArray(String key, boolean[] data) {
        return SmartDashboard.putBooleanArray(key, data);
    }

    @Override
    public boolean putBooleanArray(String key, Boolean[] data) {
        return SmartDashboard.putBooleanArray(key, data);
    }

    @Override
    public boolean setDefaultBooleanArray(String key, boolean[] defaultData) {
        return SmartDashboard.setDefaultBooleanArray(key, defaultData);
    }

    @Override
    public boolean setDefaultBooleanArray(String key, Boolean[] defaultData) {
        return SmartDashboard.setDefaultBooleanArray(key, defaultData);
    }

    @Override
    public boolean[] getBooleanArray(String key, boolean[] defaultData) {
        return SmartDashboard.getBooleanArray(key, defaultData);
    }

    @Override
    public Boolean[] getBooleanArray(String key, Boolean[] defaultData) {
        return SmartDashboard.getBooleanArray(key, defaultData);
    }

    @Override
    public boolean putNumberArray(String key, double[] data) {
        return SmartDashboard.putNumberArray(key, data);
    }

    @Override
    public boolean putNumberArray(String key, Double[] data) {
        return SmartDashboard.putNumberArray(key, data);
    }

    @Override
    public boolean setDefaultNumberArray(String key, double[] defaultData) {
        return SmartDashboard.setDefaultNumberArray(key, defaultData);
    }

    @Override
    public boolean setDefaultNumberArray(String key, Double[] defaultData) {
        return SmartDashboard.setDefaultNumberArray(key, defaultData);
    }

    @Override
    public double[] getNumberArray(String key, double[] defaultData) {
        return SmartDashboard.getNumberArray(key, defaultData);
    }

    @Override
    public Double[] getNumberArray(String key, Double[] defaultData) {
        return SmartDashboard.getNumberArray(key, defaultData);
    }

    @Override
    public boolean putStringArray(String key, String[] data) {
        return SmartDashboard.putStringArray(key, data);
    }

    @Override
    public boolean setDefaultStringArray(String key, String[] defaultData) {
        return SmartDashboard.setDefaultStringArray(key, defaultData);
    }

    @Override
    public String[] getStringArray(String key, String[] defaultData) {
        return SmartDashboard.getStringArray(key, defaultData);
    }

    @Override
    public boolean putRaw(String key, byte[] data) {
        return SmartDashboard.putRaw(key, data);
    }

    @Override
    public boolean setDefaultRaw(String key, byte[] defaultData) {
        return SmartDashboard.setDefaultRaw(key, defaultData);
    }

    @Override
    public byte[] getRaw(String key, byte[] defaultValue) {
        return SmartDashboard.getRaw(key, defaultValue);
    }

    @Override
    public void postListenerTask(Runnable task) {
        SmartDashboard.postListenerTask(task);
    }

    @Override
    public void updateValues() {
        SmartDashboard.updateValues();
    }
}
