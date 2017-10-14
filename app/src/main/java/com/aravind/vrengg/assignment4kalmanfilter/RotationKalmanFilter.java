package com.aravind.vrengg.assignment4kalmanfilter;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NullArgumentException;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.MatrixDimensionMismatchException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.NonSquareMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularMatrixException;
import org.apache.commons.math3.util.MathUtils;

/**
 * Created by Aravind on 10/14/2017.
 */

public class RotationKalmanFilter {
    private final ProcessModel processModel;
    private final MeasurementModel measurementModel;
    private RealMatrix transitionMatrix;
    private RealMatrix transitionMatrixT;
    private RealMatrix controlMatrix;
    private RealMatrix measurementMatrix;
    private RealMatrix measurementMatrixT;
    private RealVector stateEstimation;
    private RealMatrix errorCovariance;

    public RotationKalmanFilter(ProcessModel process, MeasurementModel measurement) throws NullArgumentException, NonSquareMatrixException, DimensionMismatchException, MatrixDimensionMismatchException {
        MathUtils.checkNotNull(process);
        MathUtils.checkNotNull(measurement);
        this.processModel = process;
        this.measurementModel = measurement;
        this.transitionMatrix = this.processModel.getStateTransitionMatrix();
        MathUtils.checkNotNull(this.transitionMatrix);
        this.transitionMatrixT = this.transitionMatrix.transpose();
        if(this.processModel.getControlMatrix() == null) {
            this.controlMatrix = new Array2DRowRealMatrix();
        } else {
            this.controlMatrix = this.processModel.getControlMatrix();
        }

        this.measurementMatrix = this.measurementModel.getMeasurementMatrix();
        MathUtils.checkNotNull(this.measurementMatrix);
        this.measurementMatrixT = this.measurementMatrix.transpose();
        RealMatrix processNoise = this.processModel.getProcessNoise();
        MathUtils.checkNotNull(processNoise);
        RealMatrix measNoise = this.measurementModel.getMeasurementNoise();
        MathUtils.checkNotNull(measNoise);
        if(this.processModel.getInitialStateEstimate() == null) {
            this.stateEstimation = new ArrayRealVector(this.transitionMatrix.getColumnDimension());
        } else {
            this.stateEstimation = this.processModel.getInitialStateEstimate();
        }

        if(this.transitionMatrix.getColumnDimension() != this.stateEstimation.getDimension()) {
            throw new DimensionMismatchException(this.transitionMatrix.getColumnDimension(), this.stateEstimation.getDimension());
        } else {
            if(this.processModel.getInitialErrorCovariance() == null) {
                this.errorCovariance = processNoise.copy();
            } else {
                this.errorCovariance = this.processModel.getInitialErrorCovariance();
            }

            if(!this.transitionMatrix.isSquare()) {
                throw new NonSquareMatrixException(this.transitionMatrix.getRowDimension(), this.transitionMatrix.getColumnDimension());
            } else if(this.controlMatrix != null && this.controlMatrix.getRowDimension() > 0 && this.controlMatrix.getColumnDimension() > 0 && this.controlMatrix.getRowDimension() != this.transitionMatrix.getRowDimension()) {
                throw new MatrixDimensionMismatchException(this.controlMatrix.getRowDimension(), this.controlMatrix.getColumnDimension(), this.transitionMatrix.getRowDimension(), this.controlMatrix.getColumnDimension());
            } else {
                MatrixUtils.checkAdditionCompatible(this.transitionMatrix, processNoise);
                if(this.measurementMatrix.getColumnDimension() != this.transitionMatrix.getRowDimension()) {
                    throw new MatrixDimensionMismatchException(this.measurementMatrix.getRowDimension(), this.measurementMatrix.getColumnDimension(), this.measurementMatrix.getRowDimension(), this.transitionMatrix.getRowDimension());
                } else if(measNoise.getRowDimension() != this.measurementMatrix.getRowDimension()) {
                    throw new MatrixDimensionMismatchException(measNoise.getRowDimension(), measNoise.getColumnDimension(), this.measurementMatrix.getRowDimension(), measNoise.getColumnDimension());
                }
            }
        }
    }

    public int getStateDimension() {
        return this.stateEstimation.getDimension();
    }

    public int getMeasurementDimension() {
        return this.measurementMatrix.getRowDimension();
    }

    public double[] getStateEstimation() {
        return this.stateEstimation.toArray();
    }

    public RealVector getStateEstimationVector() {
        return this.stateEstimation.copy();
    }

    public double[][] getErrorCovariance() {
        return this.errorCovariance.getData();
    }

    public RealMatrix getErrorCovarianceMatrix() {
        return this.errorCovariance.copy();
    }

    public void predict() {
        this.predict((RealVector)null);
    }

    public void predict(double[] u) throws DimensionMismatchException {
        this.predict((RealVector)(new ArrayRealVector(u, false)));
    }

    public void predict(RealVector u) throws DimensionMismatchException {
        if(u != null && u.getDimension() != this.controlMatrix.getColumnDimension()) {
            throw new DimensionMismatchException(u.getDimension(), this.controlMatrix.getColumnDimension());
        } else {
            if(u != null) {
                this.stateEstimation = u;
            }

            this.errorCovariance = this.transitionMatrix.multiply(this.errorCovariance).multiply(this.transitionMatrixT).add(this.processModel.getProcessNoise());
        }
    }

    public void correct(double[] z) throws NullArgumentException, DimensionMismatchException, SingularMatrixException {
        this.correct((RealVector)(new ArrayRealVector(z, false)));
    }

    public void correct(RealVector z) throws NullArgumentException, DimensionMismatchException, SingularMatrixException {
        MathUtils.checkNotNull(z);
        if(z.getDimension() != this.measurementMatrix.getRowDimension()) {
            throw new DimensionMismatchException(z.getDimension(), this.measurementMatrix.getRowDimension());
        } else {
            RealMatrix s = this.measurementMatrix.multiply(this.errorCovariance).multiply(this.measurementMatrixT).add(this.measurementModel.getMeasurementNoise());
            RealVector innovation = z.subtract(this.measurementMatrix.operate(this.stateEstimation));
            RealMatrix kalmanGain = (new CholeskyDecomposition(s)).getSolver().solve(this.measurementMatrix.multiply(this.errorCovariance.transpose())).transpose();
            this.stateEstimation = this.stateEstimation.add(kalmanGain.operate(innovation));
            RealMatrix identity = MatrixUtils.createRealIdentityMatrix(kalmanGain.getRowDimension());
            this.errorCovariance = identity.subtract(kalmanGain.multiply(this.measurementMatrix)).multiply(this.errorCovariance);
        }
    }
}
