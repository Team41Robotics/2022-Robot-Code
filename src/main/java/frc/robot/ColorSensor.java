package frc.robot;

import java.util.Arrays;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;


/** Custom class to handle all code for the color sensors */
public class ColorSensor {
  private ColorSensorV3 sensor;
  private int pointer;
  private int[] colorData;
  private int[] medians = {0, 0, 0};
  private int[][] buffer = new int[Constants.COLOR_BUFFER_LEN][3];

  /**
   * Intialize color sensor with REV Object
   * @param realSensor the sensor object from REV
   */
  public ColorSensor(ColorSensorV3 realSensor) {
    sensor = realSensor;
    colorData = new int[3];
    pointer = 0;
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

  /** Function to run during teleop */
  public void teleop() {
    filter();
    // System.out.println(Arrays.toString(colorData));
    
  }

  /**
   * Check the calculated value and compare to threshold
   * @return true if we are on a line, false if otherwise
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

  /**
   * Get the median value of one of the 3 colors from the buffer
   * @param color the index of the color to check (
   * 0 = red;
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


// OLD
final class ColorSensorOLD {
    private ColorSensorV3 sensorRight;
    private ColorSensorV3 sensorLeft;
    private final int BUFFER_LEN = 256;
    private int[] rightColor;
    private int[] leftColor;
    private int[][] leftBuffer = new int[BUFFER_LEN][3];
    private int[][] rightBuffer = new int[BUFFER_LEN][3];
    private int leftPointer;
    private int rightPointer;
    private int pointer;
    private int[] leftMedianList = {0, 0, 0};
    private int[] rightMedianList = {0, 0, 0};
    
    public ColorSensorOLD() {
        sensorRight = new ColorSensorV3(Port.kOnboard);
        sensorLeft = new ColorSensorV3(Port.kMXP);
        leftColor = new int[3];
        rightColor = new int[3];
        pointer = 0;
    }

        public void teleop() {
        rightColor[0] = sensorRight.getRed();
        rightColor[1] = sensorRight.getGreen();
        rightColor[2] = sensorRight.getBlue();
        int finalNum = getAdjColor(rightColor, rightBuffer, rightMedianList);
        //System.out.println(finalNum);

    }

    public boolean findLineR() {
        double threshold;
        double colorRight;
        rightColor[0] = sensorRight.getRed();
        rightColor[1] = sensorRight.getGreen();
        rightColor[2] = sensorRight.getBlue();
        if(DriverStation.getAlliance() == Alliance.Blue){
          threshold = Constants.BLUE_TAPE_RIGHT_THRESHOLD;
        } else {
          threshold = Constants.RED_TAPE_RIGHT_THRESHOLD;
        }
        colorRight = getAdjColor(rightColor, rightBuffer, rightMedianList);
        return colorRight >= threshold;
    }
    
    public boolean findLineL(){
      double threshold;
      double colorLeft;
      leftColor[0] = sensorLeft.getRed();
      leftColor[1] = sensorLeft.getGreen();
      leftColor[2] = sensorLeft.getBlue();
      if(DriverStation.getAlliance() == Alliance.Blue){
        threshold = Constants.BLUE_TAPE_LEFT_THRESHOLD;
      } else {
        threshold = Constants.RED_TAPE_LEFT_THRESHOLD;
      }
      colorLeft = getAdjColor(leftColor, leftBuffer, leftMedianList);
      return colorLeft >= threshold;
    }

    private int getAdjColor(int[] data, int[][] buffer, int[] medianList) {
      int finalNum = 1;
      int[] adjustedColorList = new int[3];
      for (int i = 0; i < 3; i++) {
        buffer[pointer][i] = data[i];
        medianList[i] = getMedian(i, buffer);
        adjustedColorList[i] = data[i] - medianList[i];
        finalNum = Math.abs(finalNum*adjustedColorList[i]);
      }
      pointer = (pointer+1) % BUFFER_LEN;      
      //System.out.println(Arrays.toString(medianList));
      return finalNum;
    }

  
    private int getMedian(int color, int[][] buffer) {
      int[] calc = new int[BUFFER_LEN];
      for (int i = 0; i < BUFFER_LEN; i++) {
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
