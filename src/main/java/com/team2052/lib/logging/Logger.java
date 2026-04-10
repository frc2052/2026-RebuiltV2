package com.team2052.lib.logging;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;

public class Logger {

    private static Map<String, Pair<BooleanLogEntry, Boolean>> booleanEntries = new HashMap<>();
    private static Map<String, Pair<DoubleLogEntry, Double>> doubleEntries = new HashMap<>();
    private static Map<String, Pair<StringLogEntry, String>> stringEntries = new HashMap<>();
    private static Map<String, Pair<StructLogEntry<StructSerializable>, StructSerializable>> structEntries = new HashMap<>();
    
    public static void start() {
        DataLogManager.start();
    }

    public static void start(String dir) {
        DataLogManager.start(dir);
    }

    public static void start(String dir, String fileName) {
        DataLogManager.start(dir, fileName);
    }

    public static void start(String dir, String filename, double period) {
        DataLogManager.start(dir, filename, period);
    }

    public static void stop() {
        DataLogManager.stop();
    }

    public static void log(String name, boolean value) {
        name = "Data Logger/" + name;
        if (!booleanEntries.containsKey(name)) {
            BooleanLogEntry entry = new BooleanLogEntry(DataLogManager.getLog(), name);
            entry.append(value);
            booleanEntries.put(name, new Pair<>(entry, value));
            return;
        }

        Pair<BooleanLogEntry, Boolean> entryPair = booleanEntries.get(name);
        if (entryPair.getSecond() == value) {
            return;
        }

        entryPair.getFirst().append(value);
        booleanEntries.put(name, new Pair<>(entryPair.getFirst(), value));
    }

    public static void log(String name, double value) {
        name = "Data Logger/" + name;
        if (!doubleEntries.containsKey(name)) {
            DoubleLogEntry entry = new DoubleLogEntry(DataLogManager.getLog(), name);
            entry.append(value);
            doubleEntries.put(name, new Pair<>(entry, value));
            return;
        }

        Pair<DoubleLogEntry, Double> entryPair = doubleEntries.get(name);
        if (entryPair.getSecond() == value) {
            return;
        }

        entryPair.getFirst().append(value);
        doubleEntries.put(name, new Pair<>(entryPair.getFirst(), value));
    }

    public static void log(String name, String value) {
        name = "Data Logger/" + name;
        if (!stringEntries.containsKey(name)) {
            StringLogEntry entry = new StringLogEntry(DataLogManager.getLog(), name);
            entry.append(value);
            stringEntries.put(name, new Pair<>(entry, value));
            return;
        }

        Pair<StringLogEntry, String> entryPair = stringEntries.get(name);
        if (entryPair.getSecond().equals(value)) {
            return;
        }

        entryPair.getFirst().append(value);
        stringEntries.put(name, new Pair<>(entryPair.getFirst(), value));
    }

    @SuppressWarnings("unchecked")
    public static <T extends StructSerializable> void log(String name, T value) {
        name = "Data Logger/" + name;
        if (!structEntries.containsKey(name)) {
            StructLogEntry<T> entry;

            switch (value.getClass().getSimpleName()) {
                case "Pose2d":
                    entry = (StructLogEntry<T>) StructLogEntry.create(DataLogManager.getLog(), name, Pose2d.struct);
                    break;
                
                default:
                    return;
            }
            entry.append(value);
            structEntries.put(name, new Pair<>((StructLogEntry<StructSerializable>) entry, value));
            return;
        }

        Pair<StructLogEntry<StructSerializable>, StructSerializable> entryPair = structEntries.get(name);
        if (entryPair.getSecond().equals(value)) {
            return;
        }

        entryPair.getFirst().append((StructSerializable) value);
        structEntries.put(name, new Pair<>(entryPair.getFirst(), value));
    }

    public static void clear() {
        booleanEntries.clear();
        doubleEntries.clear();
        stringEntries.clear();
    }

}
