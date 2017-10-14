package com.aravind.vrengg.assignment4kalmanfilter;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Iterator;

/**
 * Created by Aravind on 10/12/2017.
 */

public class MeanFilter {

    public static float DEFAULT_TIME_CONSTANT = 0.18F;

    private static final String tag = MeanFilter.class.getSimpleName();
    private ArrayDeque<float[]> values;

    protected float timeConstant;
    protected long startTime;
    protected long timestamp;
    protected int count;

    public MeanFilter() { this(DEFAULT_TIME_CONSTANT); }

    public MeanFilter(float timeConstant) {
        this.timeConstant = timeConstant;
        this.reset();

        this.values = new ArrayDeque<>();
    }

    public float[] filter(float[] data) {
        if(this.startTime == 0L) {
            this.startTime = System.nanoTime();
        }

        this.timestamp = System.nanoTime();
        float hz = (float)(this.count++) / ((float)(this.timestamp - this.startTime) / 1.0E9F);
        int filterWindow = (int)Math.ceil((double)(hz * this.timeConstant));
        this.values.addLast(Arrays.copyOf(data, data.length));

        while(this.values.size() > filterWindow) {
            this.values.removeFirst();
        }

        return this.getMean(this.values);
    }

    private float[] getMean(ArrayDeque<float[]> data) {
        float[] mean = new float[3];
        Iterator i = data.iterator();

        while(i.hasNext()) {
            float[] axis = (float[])i.next();

            for(int i1 = 0; i1 < axis.length; ++i1) {
                mean[i1] += axis[i1];
            }
        }

        for(int var6 = 0; var6 < mean.length; ++var6) {
            mean[var6] /= (float)data.size();
        }

        return mean;
    }

    public void setTimeConstant(float timeConstant) {
        this.timeConstant = timeConstant;
    }

    public void reset() {
        this.startTime = 0L;
        this.timestamp = 0L;
        this.count = 0;

        if (this.values != null)
            this.values.clear();
    }

}
