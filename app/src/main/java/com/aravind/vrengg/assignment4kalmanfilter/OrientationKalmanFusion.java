package com.aravind.vrengg.assignment4kalmanfilter;

import android.hardware.SensorManager;
import android.util.Log;

import org.apache.commons.math3.complex.Quaternion;

import java.util.Arrays;

/**
 * Created by Aravind on 10/14/2017.
 */

public class OrientationKalmanFusion extends OrientationFusion {
    private static final String tag = OrientationComplimentaryFusion.class.getSimpleName();
    private RotationKalmanFilter kalmanFilter;
    private RotationProcessModel pm;
    private RotationMeasurementModel mm;
    private volatile boolean run;
    private volatile float dt;
    private volatile float[] fusedOrientation;
    private volatile float[] acceleration;
    private volatile float[] magnetic;
    private volatile float[] gyroscope;
    private Thread thread;

    public OrientationKalmanFusion() {
        this(DEFAULT_TIME_CONSTANT);
    }

    public OrientationKalmanFusion(float timeConstant) {
        super(timeConstant);
        this.fusedOrientation = new float[3];
        this.acceleration = new float[3];
        this.magnetic = new float[3];
        this.gyroscope = new float[4];
        this.pm = new RotationProcessModel();
        this.mm = new RotationMeasurementModel();
        this.kalmanFilter = new RotationKalmanFilter(this.pm, this.mm);
    }

    public void startFusion() {
        if(!this.run && this.thread == null) {
            this.run = true;
            this.thread = new Thread(new Runnable() {
                public void run() {
                    while(OrientationKalmanFusion.this.run && !Thread.interrupted()) {
                        OrientationKalmanFusion.this.calculate();

                        try {
                            Thread.sleep(20L);
                        } catch (InterruptedException var2) {
                            Log.e(OrientationKalmanFusion.tag, "Kalman Thread Run", var2);
                        }
                    }

                }
            });
            this.thread.start();
        }

    }

    public void stopFusion() {
        if(this.run && this.thread != null) {
            this.run = false;
            this.thread.interrupt();
            this.thread = null;
        }

    }

    private float[] calculate() {
        float[] baseOrientation = this.getBaseOrientation(this.acceleration, this.magnetic);
        if(baseOrientation != null) {
            Quaternion rotationVectorAccelerationMagnetic = this.getAccelerationMagneticRotationVector(baseOrientation);
            this.initializeRotationVectorGyroscopeIfRequired(rotationVectorAccelerationMagnetic);
            this.rotationVectorGyroscope = this.getGyroscopeRotationVector(this.rotationVectorGyroscope, this.gyroscope, this.dt);
            this.dt = 0.0F;
            double[] vectorGyroscope = new double[]{(double)((float)this.rotationVectorGyroscope.getVectorPart()[0]), (double)((float)this.rotationVectorGyroscope.getVectorPart()[1]), (double)((float)this.rotationVectorGyroscope.getVectorPart()[2]), (double)((float)this.rotationVectorGyroscope.getScalarPart())};
            double[] vectorAccelerationMagnetic = new double[]{(double)((float)rotationVectorAccelerationMagnetic.getVectorPart()[0]), (double)((float)rotationVectorAccelerationMagnetic.getVectorPart()[1]), (double)((float)rotationVectorAccelerationMagnetic.getVectorPart()[2]), (double)((float)rotationVectorAccelerationMagnetic.getScalarPart())};
            this.kalmanFilter.predict(vectorGyroscope);
            this.kalmanFilter.correct(vectorAccelerationMagnetic);
            this.rotationVectorGyroscope = new Quaternion(this.kalmanFilter.getStateEstimation()[3], Arrays.copyOfRange(this.kalmanFilter.getStateEstimation(), 0, 3));
            float[] fusedVector = new float[]{(float)this.rotationVectorGyroscope.getVectorPart()[0], (float)this.rotationVectorGyroscope.getVectorPart()[1], (float)this.rotationVectorGyroscope.getVectorPart()[2], (float)this.rotationVectorGyroscope.getScalarPart()};
            float[] fusedMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(fusedMatrix, fusedVector);
            SensorManager.getOrientation(fusedMatrix, this.fusedOrientation);
            return this.fusedOrientation;
        } else {
            Log.w(tag, "Base Device Orientation could not be computed!");
            return null;
        }
    }

    protected float[] calculateFusedOrientation(float[] gyroscope, float dt, float[] acceleration, float[] magnetic) {
        this.gyroscope = gyroscope;
        this.dt += dt;
        this.acceleration = acceleration;
        this.magnetic = magnetic;
        return this.fusedOrientation;
    }
}
