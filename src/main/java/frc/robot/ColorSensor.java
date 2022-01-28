package frc.robot;

import java.util.Arrays;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/** Custom class to handle all code for the color sensors */
public class ColorSensor {
  private int pointer;
  private int[] colorData;
  private int[] medians;
  private int[][] buffer;
  private ColorSensorV3 sensor;

  /**
   * Intialize color sensor with REV Object
   * @param realSensor the sensor object from REV
   */
  public ColorSensor(ColorSensorV3 realSensor) {
    pointer = 0;
    sensor = realSensor;
    colorData = new int[3];
    medians = new int[]{0, 0, 0};
    buffer = new int[Constants.COLOR_BUFFER_LEN][3];
  }

  /**
   * Run color sensor and filter colors
   * @return the filtered color signal
   */
  private int filter() {
    colorData[0] = sensor.getRed();
    colorData[1] = sensor.getGreen();
    colorData[2] = sensor.getBlue();
    return getAdjColor();
  }

  /**
   * Filter the colors by retreiving the max between red and blue.
   * Works on both blue and red tape
   * @return The filtered color value
   */
  private int maxFilter() {
    colorData[0] = sensor.getRed();
    colorData[1] = sensor.getGreen();
    colorData[2] = sensor.getBlue();
    return Math.max(colorData[0]-medians[0], colorData[2]-medians[2]);
  }

  /** Function to run during teleop */
  public void teleop() {
    System.out.println(findLineMax());
    // System.out.println(Arrays.toString(colorData));
  }

  /**
   * Check the calculated value and compare to threshold
   * @return True if robot is on a line, false if otherwise
   */
  public boolean findLine() {
    double threshold;
    if(DriverStation.getAlliance() == Alliance.Blue){
      threshold = Constants.BLUE_TAPE_RIGHT_THRESHOLD;
    } else {
      threshold = Constants.RED_TAPE_RIGHT_THRESHOLD;
    }
    return filter() >= threshold;
  }

  /**
   * Get the threshold and check to see if the robot is currently on the tape
   * @return True if on the tape, false otherwise
   */
  public boolean findLineMax() {
    double threshold = Constants.MAX_TAPE_VALUES_THRESHOLD;
    return maxFilter() >= threshold;
  }

  /**
   * Get the calculated number based off of the r g an b values in colorData[]
   * @return the calculated value
   */
  private int getAdjColor() {
    int finalNum = 1;
    int[] adjustedColorList = new int[3];
    for (int i = 0; i < 3; i++) {
      buffer[pointer][i] = colorData[i];
      medians[i] = getMedian(i);
      adjustedColorList[i] = colorData[i] - medians[i];
      finalNum = Math.abs(finalNum*adjustedColorList[i]);
    }
    pointer = (pointer+1) % Constants.COLOR_BUFFER_LEN;      
    return finalNum;
  }

  /** Calculates the median color sensor data in preparation for running the robot */
  public void calcMedian() {
    for(int i = 0;i<256;i++){
      findLine();
    }
  }

  /**
   * Get the median value of one of the 3 colors from the buffer
   * @param color the index of the color to check
   *(0 = red;
   * 1 = green;
   * 2 = blue)
   * @return the median of the color
   */
  private int getMedian(int color) {
    int[] calc = new int[Constants.COLOR_BUFFER_LEN];
    for (int i = 0; i < Constants.COLOR_BUFFER_LEN; i++) {
      calc[i] = buffer[i][color];
    }
    Arrays.sort(calc);
      float median;
      if (calc.length % 2 == 0)
          median = ((float) calc[calc.length / 2] + (float) calc[calc.length / 2 - 1]) / 2;
      else
          median = (float) calc[calc.length / 2];
      return Math.round(median);
  }
}