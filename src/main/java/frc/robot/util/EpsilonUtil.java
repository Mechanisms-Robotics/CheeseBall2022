package frc.robot.util;

public class EpsilonUtil {
  public static boolean epsilonEquals(double value, double desiredValue, double epsilon) {
    return Math.abs(desiredValue - value) <= epsilon;
  }
}
