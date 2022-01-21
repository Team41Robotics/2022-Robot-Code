package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
    private ColorSensorV3 sensorRight;
    private ColorSensorV3 sensorLeft;
    private int[] rightColor;
    private int[] leftColor;
    private int[][] leftBuffer = new int[3][1000];
    private int[] leftBufferPointers = new int[3];
    
    public ColorSensor() {
        sensorRight = new ColorSensorV3(Port.kOnboard);
        sensorLeft = new ColorSensorV3(Port.kMXP);
        leftColor = new int[3];
        rightColor = new int[3];
        leftBufferPointers[0] = 0;
        leftBufferPointers[1] = 0;
        leftBufferPointers[2] = 0;
    }

    public boolean findLine() {
        Color colorLeft;
        int colorRight;
        int threshold;
        if(DriverStation.getAlliance() == Alliance.Blue){
          colorRight = sensorRight.getBlue();
          colorLeft = sensorLeft.getColor();

          System.out.println(colorLeft.blue);

          threshold = Constants.BLUE_TAPE_THRESHOLD;
        } else {
          colorRight = sensorRight.getRed();
          colorLeft = sensorLeft.getColor();
          threshold = Constants.RED_TAPE_THRESHOLD;
        }

        if(colorRight <= threshold && colorRight <= threshold) {
            return true;
          } else {
            return false;
          }
    }

    public void teleop() {
        leftColor[0] = sensorLeft.getRed();
        leftColor[1] = sensorLeft.getGreen();
        leftColor[2] = sensorLeft.getBlue();
        for (int i = 0; i < 4; i++) {
                leftBuffer[i][leftBufferPointers[i]] = leftColor[i];
                leftBufferPointers[i] = (leftBufferPointers[i] + 1) % leftBuffer[i].length;
            }
        /*
        leftBuffer[0][leftBufferPointers[0]] = leftColor[0];
        leftBuffer[1][leftBufferPointers[1]] = leftColor[1];
        leftBuffer[2][leftBufferPointers[2]] = leftColor[2];
        leftBufferPointers[0] = (leftBufferPointers[0] + 1) % leftBuffer[0].length;
        leftBufferPointers[1] = (leftBufferPointers[1] + 1) % leftBuffer[1].length;
        leftBufferPointers[2] = (leftBufferPointers[2] + 1) % leftBuffer[2].length;
        */
        int redMedian = getMedian(leftBuffer[0]);
        int greenMedian = getMedian(leftBuffer[1]);
        int blueMedian = getMedian(leftBuffer[2]);
        int redAdjusted = leftColor[0] - redMedian;
        int greenAdjusted = leftColor[1] - greenMedian;
        int blueAdjusted = leftColor[2] - blueMedian;
        int finalNum = Math.abs(redAdjusted*greenAdjusted*blueAdjusted);
        System.out.println(finalNum);
    }

    private int getMedian(int[] arr) {
      int[] calc = arr.clone();
      Arrays.sort(calc);
      return calc[calc.length/2];
    }
}
