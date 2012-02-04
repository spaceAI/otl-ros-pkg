/**
 * @author Takashi Ogura (t.ogura@gmail.com)
 * @license New BSD License
 * 
 */
package com.ogutti.ros.android.roomba;

import org.ros.message.geometry_msgs.Twist;
import org.ros.message.std_msgs.Bool;

import org.ros.namespace.GraphName;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.Node;

/**
 * class for Controlling a roomba by Twist msg
 * 
 */
public class RoombaControllerNode implements NodeMain {
	private Publisher<Twist> velPublisher;
	private Publisher<Bool> cleanPublisher;
	private Publisher<Bool> dockPublisher;

	@Override
	public void onShutdown(Node arg0) {
	}

	@Override
	public void onShutdownComplete(Node arg0) {
	}

	@Override
	public void onStart(Node node) {
		velPublisher = node.newPublisher("/cmd_vel", "geometry_msgs/Twist");
		cleanPublisher = node.newPublisher("/roomba/clean", "std_msgs/Bool");
		dockPublisher = node.newPublisher("/roomba/dock", "std_msgs/Bool");

	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("roomba_controller");
	}

	/**
	 * start/stop cleaning motor
	 * @param isOn if true start cleaning, false stop.
	 */
	public void publishClean(boolean isOn) {
		if (cleanPublisher != null) {
			Bool msg = new Bool();
			msg.data = isOn;
			cleanPublisher.publish(msg);
		}
	}

	/**
	 * Publish roomba's velocity (Vx, Vtheta)
	 * @param linearX twist.linear.x forward speed [m/s]
	 * @param angularZ twist.angular.z rotational speed [rad/s]
	 */
	public void publishVelocity(double linearX, double angularZ) {
		if (velPublisher != null) {
			Twist vel = new Twist();
			vel.linear.x = linearX;
			vel.angular.z = angularZ;
			velPublisher.publish(vel);
		}
	}

	/**
	 * start docking
	 */
	public void publishDock() {
		if (dockPublisher != null) {
			Bool msg = new Bool();
			msg.data = true;
			dockPublisher.publish(msg);
		}
	}
}
