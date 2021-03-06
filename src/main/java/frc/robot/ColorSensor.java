package frc.robot;

import java.util.Arrays;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
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
    * @deprecated
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
    
    /**
    * Check the calculated value and compare to threshold
    * @deprecated
    * @return True if robot is on a line, false if not
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
    * Get the calculated number based off of the r g and b values in colorData[]
    * @deprecated
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
    * Calculates the median color sensor data in preparation for running the robot
    */
    public void calcMedian() {
        for(int i = 0;i<256;i++){
            findLineMax();
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
    
    /**
    * Get the red value detected by the color sensor
    * @return The red portion of the RGB read by the color sensor
    */
    public int getRed() {
        return sensor.getRed();
    }
    
    /**
    * Add all values from this color sensor in the telemetry NetworkTable
    * @param table the base telemetry NetworkTable
    * @param name the name of this sensor to use in the table
    */
    public void telemetry(NetworkTable table, String name){
        NetworkTable lightSensorTable = table.getSubTable("light_sensors");
        
        Number[] mediansConverted = new Number[3];
        for (int i = 0; i < medians.length; i++) {
            mediansConverted[i] = medians[i];
        }
        
        NetworkTable thisSensorTable = lightSensorTable.getSubTable(name);
        thisSensorTable.getEntry("name").setString(name);
        thisSensorTable.getEntry("bias").setNumberArray(mediansConverted);
        thisSensorTable.getEntry("filter_output").setNumber(maxFilter());
        thisSensorTable.getEntry("r").setNumber(sensor.getRed());
        thisSensorTable.getEntry("g").setNumber(sensor.getGreen());
        thisSensorTable.getEntry("b").setNumber(sensor.getBlue());
    }
}