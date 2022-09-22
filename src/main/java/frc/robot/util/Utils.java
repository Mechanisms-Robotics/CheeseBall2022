package frc.robot.util;

public class Utils {
  public static boolean epsilonEquals(double value, double desiredValue, double epsilon) {
    return Math.abs(desiredValue - value) <= epsilon;
  }
}
