package monologue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * The GlobalLogged class is a utility class that provides a simple way to use Monologue's
 * logging tooling from any part of your robot code. It provides a set of log methods that
 * allow you to log data to the NetworkTables and DataLog.
 * 
 * @see Monologue
 * @see LogLevel
 */
class GlobalLogged {
  static String ROOT_PATH = "";

  static void setRootPath(String rootPath) {
    ROOT_PATH = NetworkTable.normalizeKey(rootPath, true);
  }

  /**
    * Logs a boolean using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static boolean log(String entryName, boolean value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a boolean using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static boolean log(String entryName, boolean value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a int using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static int log(String entryName, int value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a int using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static int log(String entryName, int value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a long using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static long log(String entryName, long value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a long using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static long log(String entryName, long value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a float using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static float log(String entryName, float value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a float using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static float log(String entryName, float value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a double using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static double log(String entryName, double value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a double using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static double log(String entryName, double value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a String using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static String log(String entryName, String value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a String using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static String log(String entryName, String value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a byte[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static byte[] log(String entryName, byte[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a byte[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static byte[] log(String entryName, byte[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a boolean[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static boolean[] log(String entryName, boolean[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a boolean[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static boolean[] log(String entryName, boolean[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a int[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static int[] log(String entryName, int[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a int[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static int[] log(String entryName, int[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a long[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static long[] log(String entryName, long[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a long[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static long[] log(String entryName, long[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a float[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static float[] log(String entryName, float[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a float[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static float[] log(String entryName, float[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a double[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static double[] log(String entryName, double[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a double[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static double[] log(String entryName, double[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a String[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static String[] log(String entryName, String[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a String[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static String[] log(String entryName, String[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }


  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static <R extends StructSerializable> R log(String entryName, R value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static <R extends StructSerializable> R log(String entryName, R value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static <R extends StructSerializable> R[] log(String entryName, R[] value) {
    return log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static <R extends StructSerializable> R[] log(String entryName, R[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    */
  public static <R> R log(String entryName, Struct<R> struct, R value) {
    return log(entryName, struct, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static <R> R log(String entryName, Struct<R> struct, R value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, struct, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.putStruct(entryName, struct, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.putStruct(entryName, struct, value, level);
      }
    }

    return value;
  }

  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    */
  public static <R> R[] log(String entryName, Struct<R> struct, R[] value) {
    return log(entryName, struct, value, LogLevel.DEFAULT);
  }
  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static <R> R[] log(String entryName, Struct<R> struct, R[] value, LogLevel level) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureCalls.add(() -> GlobalLogged.log(entryNameFinal, struct, value, level));
      return value;
    }
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.putStructArray(entryName, struct, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.putStructArray(entryName, struct, value, level);
      }
    }

    return value;
  }

  /**
    * Logs a Sendable using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void publishSendable(String entryName, Sendable value) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) return;
    entryName = NetworkTable.normalizeKey(entryName, true);
    Monologue.ntLogger.addSendable(entryName, value);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      NetworkTablesJNI.startEntryDataLog(
        NetworkTableInstance.getDefault().getHandle(),
        DataLogManager.getLog(),
        entryName,
        Monologue.dataLogger.prefix + entryName
      );
    }
  }
}
