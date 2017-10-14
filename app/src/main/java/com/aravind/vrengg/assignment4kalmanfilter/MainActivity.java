package com.aravind.vrengg.assignment4kalmanfilter;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String tag = MainActivity.class.getSimpleName();

    private float[] fusedOrientation = new float[3];
    private float[] acceleration = new float[4];
    private float[] magnetic = new float[3];
    private float[] rotation = new float[3];
    private float blendValue = 0;

    private Handler handler;
    private Runnable runnable;

    private TextView tvXAxis;
    private TextView tvYAxis;
    private TextView tvZAxis;
    private EditText etBlendValue;

    private OrientationFusion orientationFusion;
    private MeanFilter meanFilter;

    private SensorManager sensorManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);
        meanFilter = new MeanFilter();
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        initUI();
    }

    @Override
    public void onResume() {
        super.onResume();

        reset();

        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        // Register for sensor updates.
        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);

        // Register for sensor updates.
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);

        handler.post(runnable);
    }

    @Override
    public void onPause() {
        super.onPause();

        sensorManager.unregisterListener(this);
        handler.removeCallbacks(runnable);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            // Android reuses events, so you probably want a copy
            System.arraycopy(event.values, 0, acceleration, 0, event.values.length);
            orientationFusion.setAcceleration(acceleration);
        } else  if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            // Android reuses events, so you probably want a copy
            System.arraycopy(event.values, 0, magnetic, 0, event.values.length);
            orientationFusion.setMagneticField(this.magnetic);
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            // Android reuses events, so you probably want a copy
            System.arraycopy(event.values, 0, rotation, 0, event.values.length);
            // Filter the rotation
            fusedOrientation = orientationFusion.filter(this.rotation);

            //fusedOrientation = meanFilter.filter(fusedOrientation);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {}

    private void reset() {
        orientationFusion = new OrientationComplimentaryFusion();
        //orientationFusion.setTimeConstant(0.5f);    // default
        ((OrientationComplimentaryFusion) orientationFusion).alpha = blendValue;
        etBlendValue.setText(String.valueOf(blendValue));

        handler = new Handler();
        runnable = new Runnable() {
            @Override
            public void run() {
                handler.postDelayed(this, 100);
                updateText();
            }
        };
    }

    private void updateText() {
        tvXAxis.setText(String.format("%.2f", Math.toDegrees(fusedOrientation[0])));
        tvYAxis.setText(String.format("%.2f", Math.toDegrees(fusedOrientation[1])));
        tvZAxis.setText(String.format("%.2f", Math.toDegrees(fusedOrientation[2])));
    }

    private void initUI() {
        tvXAxis = (TextView) this.findViewById(R.id.value_x_axis_calibrated);
        tvYAxis = (TextView) this.findViewById(R.id.value_y_axis_calibrated);
        tvZAxis = (TextView) this.findViewById(R.id.value_z_axis_calibrated);

        etBlendValue = (EditText) this.findViewById(R.id.edittext_blend_value);
        etBlendValue.setOnFocusChangeListener(new View.OnFocusChangeListener() {
            @Override
            public void onFocusChange(View view, boolean hasFocus) {
                if (!hasFocus) {
                    try {
                        float val = Float.valueOf(etBlendValue.getText().toString());

                        if (blendValue != val) {
                            blendValue = val;
                            if (blendValue < 0)
                                blendValue = 0;
                            else if (blendValue > 1)
                                blendValue = 1;

                            reset();
                        }

                    } catch (Exception ex) {
                        ex.printStackTrace();
                    }
                }
            }
        });
    }
}
