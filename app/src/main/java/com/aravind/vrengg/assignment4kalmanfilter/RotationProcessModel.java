package com.aravind.vrengg.assignment4kalmanfilter;

import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * Created by Aravind on 10/14/2017.
 */

public class RotationProcessModel implements ProcessModel {
    private RealMatrix stateTransitionMatrix = new Array2DRowRealMatrix(new double[][]{{1.0D, 0.0D, 0.0D, 0.0D}, {0.0D, 1.0D, 0.0D, 0.0D}, {0.0D, 0.0D, 1.0D, 0.0D}, {0.0D, 0.0D, 0.0D, 1.0D}});
    private RealMatrix processNoiseCovMatrix = new Array2DRowRealMatrix(new double[][]{{1.0D, 0.0D, 0.0D, 0.0D}, {0.0D, 1.0D, 0.0D, 0.0D}, {0.0D, 0.0D, 1.0D, 0.0D}, {0.0D, 0.0D, 0.0D, 1.0D}});
    private RealVector initialStateEstimateVector = new ArrayRealVector(new double[]{0.0D, 0.0D, 0.0D, 0.0D});
    private RealMatrix initialErrorCovMatrix = new Array2DRowRealMatrix(new double[][]{{0.1D, 0.0D, 0.0D, 0.0D}, {0.0D, 0.1D, 0.0D, 0.0D}, {0.0D, 0.0D, 0.1D, 0.0D}, {0.0D, 0.0D, 0.0D, 0.1D}});
    private RealMatrix controlMatrix = new Array2DRowRealMatrix(new double[][]{{1.0D, 0.0D, 0.0D, 0.0D}, {0.0D, 1.0D, 0.0D, 0.0D}, {0.0D, 0.0D, 1.0D, 0.0D}, {0.0D, 0.0D, 0.0D, 1.0D}});

    public RotationProcessModel() {
    }

    public RealMatrix getStateTransitionMatrix() {
        this.stateTransitionMatrix = new Array2DRowRealMatrix(new double[][]{{1.0D, 0.0D, 0.0D, 0.0D}, {0.0D, 1.0D, 0.0D, 0.0D}, {0.0D, 0.0D, 1.0D, 0.0D}, {0.0D, 0.0D, 0.0D, 1.0D}});
        return this.stateTransitionMatrix;
    }

    public RealMatrix getControlMatrix() {
        return this.controlMatrix;
    }

    public RealMatrix getProcessNoise() {
        return this.processNoiseCovMatrix;
    }

    public RealVector getInitialStateEstimate() {
        return this.initialStateEstimateVector;
    }

    public RealMatrix getInitialErrorCovariance() {
        return this.initialErrorCovMatrix;
    }
}
