package com.aravind.vrengg.assignment4kalmanfilter;

import android.hardware.SensorManager;

import org.apache.commons.math3.complex.Quaternion;

import java.util.Arrays;

/**
 * Created by Aravind on 10/12/2017.
 */

public abstract class OrientationFusion {

    private static final float EPSILON = 1.0E-9F;
    private static final float NS2S = 1.0E-9F;
    private static final String tag = OrientationFusion.class.getSimpleName();
    public static float DEFAULT_TIME_CONSTANT = 0.18F;
    public float timeConstant;
    protected Quaternion rotationVectorGyroscope;
    private float[] acceleration;
    private boolean accelerationUpdated;
    private float[] magnetic;
    private boolean magneticUpdated;
    private long timestamp;

    public OrientationFusion() {
        this(DEFAULT_TIME_CONSTANT);
    }

    public OrientationFusion(float timeConstant) {
        this.timeConstant = timeConstant;
        this.reset();
    }

    public float[] filter(float[] values) {
        return this.getFusedOrientation(values);
    }

    public void reset() {
        this.accelerationUpdated = false;
        this.magneticUpdated = false;
        this.timestamp = 0L;
        this.magnetic = new float[3];
        this.acceleration = new float[3];
        this.rotationVectorGyroscope = null;
    }

    public void setAcceleration(float[] acceleration) {
        this.acceleration = Arrays.copyOf(acceleration, acceleration.length);
        this.accelerationUpdated = true;
    }

    public void setMagneticField(float[] magnetic) {
        this.magnetic = magnetic;
        this.magneticUpdated = true;
    }

    public void setTimeConstant(float timeConstant) {
        this.timeConstant = timeConstant;
    }

    public abstract void startFusion();

    public abstract void stopFusion();

    protected abstract float[] calculateFusedOrientation(float[] var1, float var2, float[] var3, float[] var4);

    protected Quaternion getAccelerationMagneticRotationVector(float[] orientation) {
        double c1 = Math.cos((double)(orientation[0] / 2.0F));
        double s1 = Math.sin((double)(orientation[0] / 2.0F));
        double c2 = Math.cos((double)(-orientation[1] / 2.0F));
        double s2 = Math.sin((double)(-orientation[1] / 2.0F));
        double c3 = Math.cos((double)(orientation[2] / 2.0F));
        double s3 = Math.sin((double)(orientation[2] / 2.0F));
        double c1c2 = c1 * c2;
        double s1s2 = s1 * s2;
        double w = c1c2 * c3 - s1s2 * s3;
        double x = c1c2 * s3 + s1s2 * c3;
        double y = s1 * c2 * c3 + c1 * s2 * s3;
        double z = c1 * s2 * c3 - s1 * c2 * s3;
        return new Quaternion(w, z, x, y);
    }

    protected float[] getBaseOrientation(float[] acceleration, float[] magnetic) {
        float[] rotationMatrix = new float[9];
        if(SensorManager.getRotationMatrix(rotationMatrix, (float[])null, acceleration, magnetic)) {
            float[] baseOrientation = new float[3];
            SensorManager.getOrientation(rotationMatrix, baseOrientation);
            return baseOrientation;
        } else {
            return null;
        }
    }

    protected Quaternion getGyroscopeRotationVector(Quaternion previousRotationVector, float[] rateOfRotation, float dt) {
        float magnitude = (float)Math.sqrt(Math.pow((double)rateOfRotation[0], 2.0D) + Math.pow((double)rateOfRotation[1], 2.0D) + Math.pow((double)rateOfRotation[2], 2.0D));
        if(magnitude > 1.0E-9F) {
            rateOfRotation[0] /= magnitude;
            rateOfRotation[1] /= magnitude;
            rateOfRotation[2] /= magnitude;
        }

        float thetaOverTwo = magnitude * dt / 2.0F;
        float sinThetaOverTwo = (float)Math.sin((double)thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos((double)thetaOverTwo);
        double[] deltaVector = new double[]{(double)(sinThetaOverTwo * rateOfRotation[0]), (double)(sinThetaOverTwo * rateOfRotation[1]), (double)(sinThetaOverTwo * rateOfRotation[2]), (double)cosThetaOverTwo};
        return previousRotationVector.multiply(new Quaternion(deltaVector[3], Arrays.copyOfRange(deltaVector, 0, 3)));
    }

    protected void initializeRotationVectorGyroscopeIfRequired(Quaternion rotationVectorAccelerationMagnetic) {
        this.rotationVectorGyroscope = new Quaternion(rotationVectorAccelerationMagnetic.getScalarPart(), rotationVectorAccelerationMagnetic.getVectorPart());
    }

    private float[] getFusedOrientation(float[] gyroscope) {
        long timestamp = System.nanoTime();
        if(this.accelerationUpdated && this.magneticUpdated) {
            if(this.timestamp == 0L) {
                this.timestamp = System.nanoTime();
            }

            float dt = (float)(timestamp - this.timestamp) * 1.0E-9F;
            this.timestamp = timestamp;
            return this.calculateFusedOrientation(gyroscope, dt, this.acceleration, this.magnetic);
        } else {
            return new float[3];
        }
    }

}
