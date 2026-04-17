package frc319.lib.util;

public class RollingAverage {

    private final double[] buffer;
    private int index =0;
    private int count =0;
    private double sum = 0.0;

    public RollingAverage(int size){
        buffer = new double[size];
    }

    public void add(double value){
        sum -= buffer[index];

        buffer[index]=value;
        sum+=value;

        index = (index+1) % buffer.length;

        if(count < buffer.length){
            count++;
        }


    }

    public double getAverage(){
        if (count == 0) return 0.0;
        return sum/count;
    }
    
}
