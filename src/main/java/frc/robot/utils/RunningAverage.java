package frc.robot.utils;

public class RunningAverage {
    private int writeIndex;
    private double[] dataArray;
    private int numDataReadings;
    private double tolerance;
    private double runningAverage;
    private boolean isFirstData;

    public RunningAverage(int numReadings) {
        this(numReadings, null);
    }

    public RunningAverage(int numReadings, Double tolerance) {
        numDataReadings = numReadings;
        reset();
        this.tolerance = tolerance != null ? tolerance : Double.MAX_VALUE;
    }

    public void reset() {
        dataArray = new double[numDataReadings];
        isFirstData = true;
        writeIndex = 0;
    }

    public double calculate(double num) {
        add(num);
        return runningAverage;
    }

    public void add(double incomingNum) {
        if (!isFirstData && Math.abs(incomingNum - runningAverage) > tolerance)
            return;

        dataArray[writeIndex] = incomingNum;

        runningAverage = calculateRunningAvg();
        if (writeIndex >= numDataReadings - 1) {
            resetWriteIndex();
        } else
            writeIndex++;
    }

    private double calculateRunningAvg() {
        double num = 0;
        for (double val : dataArray)
            num += val;
        return num / (isFirstData ? writeIndex + 1 : numDataReadings);
    }

    public double get() {
        return runningAverage;
    }

    private void resetWriteIndex() {
        writeIndex = 0;
        isFirstData = false;
    }

}