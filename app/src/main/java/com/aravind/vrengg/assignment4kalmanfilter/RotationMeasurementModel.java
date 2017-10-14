package com.aravind.vrengg.assignment4kalmanfilter;

import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * Created by Aravind on 10/14/2017.
 */

public class RotationMeasurementModel implements MeasurementModel {
    private double noiseCoefficient = 0.001D;
    private RealMatrix measurementMatrix = new Array2DRowRealMatrix(new double[][]{{1.0D, 0.0D, 0.0D, 0.0D}, {0.0D, 1.0D, 0.0D, 0.0D}, {0.0D, 0.0D, 1.0D, 0.0D}, {0.0D, 0.0D, 0.0D, 1.0D}});
    private RealMatrix measurementNoise;

    public RotationMeasurementModel() {
        this.measurementNoise = new Array2DRowRealMatrix(new double[][]{{this.noiseCoefficient, 0.0D, 0.0D, 0.0D}, {0.0D, this.noiseCoefficient, 0.0D, 0.0D}, {0.0D, 0.0D, this.noiseCoefficient, 0.0D}, {0.0D, 0.0D, 0.0D, this.noiseCoefficient}});
    }

    public RealMatrix getMeasurementMatrix() {
        return this.measurementMatrix;
    }

    public RealMatrix getMeasurementNoise() {
        return this.measurementNoise;
    }
}
