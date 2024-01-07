package frc.robot.utils;

public class DataStreamFilter {
    private int writeIndex = 0;
    private double[] dataArray;
    private int numDataReadings;
    private double tolerance;
    private double runningAverage;
    private boolean isFirstData = true;

    public DataStreamFilter(int numReadings) {
        this(numReadings, null);
    }

    public DataStreamFilter(int numReadings, Double tolerance) {
        numDataReadings = numReadings;
        dataArray = new double[numDataReadings];

        this.tolerance = tolerance != null ? tolerance : Double.MAX_VALUE;
    }

    public double calculate(double num) {
        addDouble(num);
        return runningAverage;
    }

    public void addDouble(double incomingNum) {
        if (!isFirstData && Math.abs(incomingNum - runningAverage) > tolerance)
            return;

        dataArray[writeIndex] = incomingNum;

        runningAverage = getRunningAvg();
        if (writeIndex >= numDataReadings - 1) {
            resetWriteIndex();
        } else
            writeIndex++;
    }

    public double getRunningAvg() {
        double num = 0;
        for (double val : dataArray)
            num += val;
        return num / (isFirstData ? writeIndex + 1 : numDataReadings);
    }

    private void resetWriteIndex() {
        writeIndex = 0;
        isFirstData = false;
    }

}