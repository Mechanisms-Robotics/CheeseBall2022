package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;

/** This class contains all the code to interface with the TCS34725 via I2C */
public class TCS34725 {
  // Device address
  private static final int I2C_DEVICE_ADDRESS = 0x29;

  // Device ID
  private static final int DEVICE_ID = 0x44;

  // Command bit
  private static final int CMD_BIT = 0x80;

  // Write registers
  private static final int ENABLE_REGISTER = 0x00;
  private static final int ATIME_REGISTER = 0x01;
  private static final int GAIN_REGISTER = 0x0F;

  // Read registers
  private static final int ID_REGISTER = 0x12;
  private static final int C_REGISTER = 0x14;
  private static final int R_REGISTER = 0x16;
  private static final int G_REGISTER = 0x18;
  private static final int B_REGISTER = 0x1A;

  // Enable register bitmasks
  private static final int PON = 0x01;
  private static final int AEN = 0x02;

  // Gain settings enum
  public enum GainSettings {
    ONE_TIMES(0x00),
    FOUR_TIMES(0x01),
    SIXTEEN_TIMES(0x02),
    SIXTY_TIMES(0x03);

    public final int value;

    GainSettings(int value) {
      this.value = value;
    }
  }

  // Sensor and buffer
  private final I2C sensor;
  private final ByteBuffer buffer = ByteBuffer.allocate(10);

  /** Constructs a TCS34725 object */
  public TCS34725(I2C.Port port) {
    // Initialize the sensor
    this.sensor = new I2C(port, I2C_DEVICE_ADDRESS);

    // Initialize the buffer
    buffer.order(ByteOrder.LITTLE_ENDIAN);

    // Power on the sensor and enable the ADCs
    sensor.write(CMD_BIT | ENABLE_REGISTER, PON | AEN);

    // Configure integration time to 2.4 ms
    sensor.write(CMD_BIT | ATIME_REGISTER, (int) (12 / 5 + 1));

    // Read product id
    buffer.clear();
    sensor.read(CMD_BIT | ID_REGISTER, 1, buffer);
    byte id = buffer.get();

    // Check if the product id is valid
    if (id != DEVICE_ID) {
      DriverStation.reportWarning(
          "Expected product id to be " + hex(DEVICE_ID) + " but got " + hex(id) + " instead!",
          true);
    }
  }

  /** Get the RGB value the color sensor detects */
  public List<Double> getRGB() {
    // Get the raw RGBC register values
    List<Integer> rawRGBC = getRawRGBC();

    // Calculate actual RGB values and return them
    return Arrays.asList(
        rawRGBC.get(0) / rawRGBC.get(3) * 255.0,
        rawRGBC.get(1) / rawRGBC.get(3) * 255.0,
        rawRGBC.get(2) / rawRGBC.get(3) * 255.0);
  }

  /** Set the gain of the sensor to a specified setting */
  public void setGain(GainSettings gainSettings) {
    // Write the gain setting value to the GAIN_REGISTER
    sensor.write(CMD_BIT | GAIN_REGISTER, gainSettings.value);
  }

  /** Read the raw RGBC values from the sensor */
  private List<Integer> getRawRGBC() {
    // Read the value of the C register
    buffer.clear();
    sensor.read(CMD_BIT | C_REGISTER, 2, buffer);
    int clear = buffer.get();

    // Read the value of the R register
    buffer.clear();
    sensor.read(CMD_BIT | R_REGISTER, 2, buffer);
    int red = buffer.get();

    // Read the value of the G register
    buffer.clear();
    sensor.read(CMD_BIT | G_REGISTER, 2, buffer);
    int green = buffer.get();

    // Read the value of the B register
    buffer.clear();
    sensor.read(CMD_BIT | B_REGISTER, 2, buffer);
    int blue = buffer.get();

    // Return the values in an array
    return Arrays.asList(red, green, blue, clear);
  }

  /** Format string as a hex number */
  private static String hex(int hex) {
    return String.format("0x%02X", hex);
  }
}
