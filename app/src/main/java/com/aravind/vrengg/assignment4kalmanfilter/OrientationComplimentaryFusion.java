package com.aravind.vrengg.assignment4kalmanfilter;

import android.hardware.SensorManager;
import android.util.Log;

import org.apache.commons.math3.complex.*;

/**
 * Created by Aravind on 10/12/2017.
 */

public class OrientationComplimentaryFusion extends OrientationFusion {

    private static final String tag = OrientationComplimentaryFusion.class.getSimpleName();

    public float alpha;

    public OrientationComplimentaryFusion() {
        this(DEFAULT_TIME_CONSTANT);
    }

    public OrientationComplimentaryFusion(float timeConstant) {
        super(timeConstant);
    }

    protected float[] calculateFusedOrientation(float[] gyroscope, float dt, float[] acceleration, float[] magnetic) {
        float[] baseOrientation = this.getBaseOrientation(acceleration, magnetic);
        if(baseOrientation != null) {
            float oneMinusAlpha = 1.0F - alpha;
            Quaternion rotationVectorAccelerationMagnetic = this.getAccelerationMagneticRotationVector(baseOrientation);
            this.initializeRotationVectorGyroscopeIfRequired(rotationVectorAccelerationMagnetic);
            this.rotationVectorGyroscope = this.getGyroscopeRotationVector(this.rotationVectorGyroscope, gyroscope, dt);
            Quaternion scaledRotationVectorAccelerationMagnetic = rotationVectorAccelerationMagnetic.multiply((double)oneMinusAlpha);
            Quaternion scaledRotationVectorGyroscope = this.rotationVectorGyroscope.multiply((double)alpha);
            this.rotationVectorGyroscope = scaledRotationVectorGyroscope.add(scaledRotationVectorAccelerationMagnetic);
            float[] fusedVector = new float[]{(float)this.rotationVectorGyroscope.getVectorPart()[0], (float)this.rotationVectorGyroscope.getVectorPart()[1], (float)this.rotationVectorGyroscope.getVectorPart()[2], (float)this.rotationVectorGyroscope.getScalarPart()};
            float[] fusedMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(fusedMatrix, fusedVector);
            float[] fusedOrientation = new float[3];
            SensorManager.getOrientation(fusedMatrix, fusedOrientation);
            return fusedOrientation;
        } else {
            Log.w(tag, "Base Device Orientation could not be computed!");
            return null;
        }
    }

    public void startFusion() {
    }

    public void stopFusion() {
    }

}
