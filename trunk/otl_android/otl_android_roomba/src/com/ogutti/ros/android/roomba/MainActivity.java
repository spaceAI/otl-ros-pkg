/**
 * @author Takashi Ogura <t.ogura@gmail.com>
 * @license New BSD License
 */

package com.ogutti.ros.android.roomba;

import java.text.DecimalFormat;
import java.util.List;
import android.os.Bundle;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;
import android.widget.ToggleButton;

import org.ros.address.InetAddressFactory;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.views.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import com.ogutti.ros.android.roomba.R;
import org.ros.message.geometry_msgs.Twist;

/**
 * MainActivity of this App (roomba controller)
 * 
 * @author t.ogura@gmail.com (OTL)
 */
public class MainActivity extends RosActivity implements SensorEventListener {

	private RosTextView<Twist> rosTextView;
	private SensorManager sensorManager;
	private RoombaControllerNode controllerNode;
	
	private static final double LINEAR_VELOCITY_RATE  = 0.05;
	private static final double ANGULAR_VELOCITY_RATE = 0.1;

	/**
	 * 1.0 means max speed, 0.0 means stop always.
	 */
	private double speedRate;

	public MainActivity() {
		super("Roomba Controller", "Roomba Controller");
		speedRate = 0.5;
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		rosTextView = (RosTextView<Twist>) findViewById(R.id.text);
		rosTextView.setTopicName("/cmd_vel");
		rosTextView.setMessageType("geometry_msgs/Twist");
		rosTextView
				.setMessageToStringCallable(new MessageCallable<String, Twist>() {
					@Override
					public String call(Twist message) {
						DecimalFormat df = new DecimalFormat();
						df.setMaximumFractionDigits(2);

						return "vel_x:\n" + df.format(message.linear.x) + "\n"
								+ "vel_theta:\n" + df.format(message.angular.z);
					}
				});
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		// create ROS nodes
		controllerNode = new RoombaControllerNode();
		NodeConfiguration nodeConfiguration = NodeConfiguration
				.newPublic(InetAddressFactory.newNonLoopback().getHostName());
		nodeConfiguration.setMasterUri(this.getMasterUri());
		nodeMainExecutor.execute(controllerNode, nodeConfiguration);
		nodeMainExecutor.execute(rosTextView, nodeConfiguration);

		// set callback for accelerometer
		List<Sensor> sensors = sensorManager
				.getSensorList(Sensor.TYPE_ACCELEROMETER);
		if (sensors.size() > 0) {
			Sensor accelerometer = sensors.get(0);
			sensorManager.registerListener(this, accelerometer,
					SensorManager.SENSOR_DELAY_FASTEST);
		} else {
			android.util.Log.v("MainActivity", "NOT found sensor!");
		}
		
		// set toggle button's callback
		ToggleButton tb = (ToggleButton) findViewById(R.id.toggleButton1);
		tb.setOnCheckedChangeListener(new OnCheckedChangeListener() {
			@Override
			public void onCheckedChanged(CompoundButton buttonView,
					boolean isChecked) {
				controllerNode.publishClean(isChecked);
			}
		});

		// set seekbar callback. 
		SeekBar seekBar = (SeekBar) findViewById(R.id.seekBar1);
		final TextView tv1 = (TextView) findViewById(R.id.textView1);
		seekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				tv1.setText("Speed:" + progress + "%");
				speedRate = progress * 0.01f;
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
			}
		});
		Button dockButton = (Button) findViewById(R.id.button1);
		dockButton.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(android.view.View v) {
				controllerNode.publishDock();
			}
		});

	}

	@Override
	protected void onResume() {
		super.onResume();

	}

	@Override
	protected void onStop() {
		super.onStop();
		sensorManager.unregisterListener(this);
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	/**
	 * callback of sensor change(accelerometer)
	 */
	@Override
	public void onSensorChanged(SensorEvent event) {
		if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
			double linearX = event.values[1] * -LINEAR_VELOCITY_RATE * speedRate;
			double angularZ = event.values[0] * ANGULAR_VELOCITY_RATE * speedRate;
			if (Math.abs(linearX) < LINEAR_VELOCITY_RATE) {
				linearX = 0.0d;
			}
			if (Math.abs(angularZ) < ANGULAR_VELOCITY_RATE) {
				angularZ = 0.0d;
			}
			controllerNode.publishVelocity(linearX, angularZ);
		}
	}
}
