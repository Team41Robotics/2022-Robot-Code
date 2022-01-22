package frc.robot;

import java.util.Arrays;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor {
    private ColorSensorV3 sensorRight;
    private ColorSensorV3 sensorLeft;
    private int[] rightColor;
    private int[] leftColor;
    private int[][] leftBuffer = new int[3][1000];
    private int[][] rightBuffer = new int[3][1000];
    private int[] leftBufferPointers = new int[3];
    private int[] rightBufferPointers = new int[3];
    private int[] leftMedianList = {797, 1300, 527};
    private int[] rightMedianList = {1633, 2786, 1186};
    private int[] adjustedColorList = new int[3];
    
    public ColorSensor() {
        sensorRight = new ColorSensorV3(Port.kOnboard);
        sensorLeft = new ColorSensorV3(Port.kMXP);
        leftColor = new int[3];
        rightColor = new int[3];
        leftBufferPointers[0] = 0;
        leftBufferPointers[1] = 0;
        leftBufferPointers[2] = 0;
    }

    public boolean findLineR() {
        int threshold;
        int colorRight;
        rightColor[0] = sensorRight.getRed();
        rightColor[1] = sensorRight.getGreen();
        rightColor[2] = sensorRight.getBlue();
        if(DriverStation.getAlliance() == Alliance.Blue){
          threshold = Constants.BLUE_TAPE_RIGHT_THRESHOLD;
        } else {
          threshold = Constants.RED_TAPE_RIGHT_THRESHOLD;
        }
        colorRight = getAdjColor(rightColor, rightBufferPointers, rightBuffer, rightMedianList);
        System.out.println(colorRight);
        if(colorRight >= threshold) {
            return true;
          } else {
            return false;
          }
    }
    public boolean findLineL(){
      int threshold;
      int colorLeft;
      leftColor[0] = sensorLeft.getRed();
      leftColor[1] = sensorLeft.getGreen();
      leftColor[2] = sensorLeft.getBlue();
      if(DriverStation.getAlliance() == Alliance.Blue){
        threshold = Constants.BLUE_TAPE_LEFT_THRESHOLD;
      } else {
        threshold = Constants.RED_TAPE_LEFT_THRESHOLD;
      }
      colorLeft = getAdjColor(leftColor, leftBufferPointers, leftBuffer, leftMedianList);
      System.out.print(colorLeft);
      System.out.print("\t\t\t");
      if (colorLeft >= threshold){
        return true;
      }else{
        return false;
      }
      
    }
    public void teleop() {
        leftColor[0] = sensorLeft.getRed();
        leftColor[1] = sensorLeft.getGreen();
        leftColor[2] = sensorLeft.getBlue();
        int finalNum = getAdjColor(leftColor, leftBufferPointers, leftBuffer, leftMedianList);
        System.out.println(finalNum);
        /*
        leftBuffer[0][leftBufferPointers[0]] = leftColor[0];
        leftBuffer[1][leftBufferPointers[1]] = leftColor[1];
        leftBuffer[2][leftBufferPointers[2]] = leftColor[2];
        leftBufferPointers[0] = (leftBufferPointers[0] + 1) % leftBuffer[0].length;
        leftBufferPointers[1] = (leftBufferPointers[1] + 1) % leftBuffer[1].length;
        leftBufferPointers[2] = (leftBufferPointers[2] + 1) % leftBuffer[2].length;
        
        int redMedian = getMedian(leftBuffer[0]);
        int greenMedian = getMedian(leftBuffer[1]);
        int blueMedian = getMedian(leftBuffer[2]);
        int redAdjusted = leftColor[0] - redMedian;
        int greenAdjusted = leftColor[1] - greenMedian;
        int blueAdjusted = leftColor[2] - blueMedian;
        int finalNum = Math.abs(redAdjusted*greenAdjusted*blueAdjusted);
        */
    }

    private int getAdjColor(int[] data, int[] pointers, int[][] buffer, int[] medianList) {
      int finalNum = 1;
      for (int i = 0; i < 3; i++) {
        buffer[i][pointers[i]] = leftColor[i];
        pointers[i] = (pointers[i] + 1) % buffer[i].length;
        medianList[i] = getMedian(buffer[i]);
        adjustedColorList[i] = data[i] - medianList[i];
        finalNum = Math.abs(finalNum*adjustedColorList[i]);
      }
      //System.out.println(Arrays.toString(medianList));
      return finalNum;
    }

    private int getMedian(int[] arr) {
      int[] calc = arr.clone();
      Arrays.sort(calc);
        float median;
        if (calc.length % 2 == 0)
            median = ((float) calc[calc.length / 2] + (float) calc[calc.length / 2 - 1]) / 2;
        else
            median = (float) calc[calc.length / 2];
        return Math.round(median);
    }
}
