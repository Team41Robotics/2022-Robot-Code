package frc.robot;

import java.util.Arrays;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor {
    private ColorSensorV3 sensorRight;
    private ColorSensorV3 sensorLeft;
    private final int BUFFER_LEN = 1024;
    private int[] rightColor;
    private int[] leftColor;
    private int[][] leftBuffer = new int[BUFFER_LEN][3];
    private int[][] rightBuffer = new int[BUFFER_LEN][3];
    private int pointer;
    private int[] leftMedianList = {0, 0, 0};
    private int[] rightMedianList = {0, 0, 0};
    
    public ColorSensor() {
        sensorRight = new ColorSensorV3(Port.kOnboard);
        sensorLeft = new ColorSensorV3(Port.kMXP);
        leftColor = new int[3];
        rightColor = new int[3];
        pointer = 0;
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
        if(colorRight >= threshold) {
            return true;
          } else {
            return false;
          }
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
      if (colorLeft >= threshold){
        return true;
      }else{
        return false;
      }
      
    }

    public void teleop() {
        rightColor[0] = sensorRight.getRed();
        rightColor[1] = sensorRight.getGreen();
        rightColor[2] = sensorRight.getBlue();
        //int finalNum = getAdjColor(leftColor, leftBufferPointers, leftBuffer, leftMedianList);
        int finalNum = getAdjColor(rightColor, rightBuffer, rightMedianList);
        //System.out.println(rightColor);
        System.out.println(finalNum);

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
