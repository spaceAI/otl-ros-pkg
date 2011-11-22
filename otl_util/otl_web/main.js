function main() {
  connectToROS("roomba.dyndns.tv", "192.168.11.4");
  joy_init();
//  connectToROS("mtm07otl.ddo.jp", "192.168.0.4");
//  connectToROS("mtm07otl.ddo.jp", "127.0.0.1");
  
  connection.setOnClose(function (e) {
			  log('connection closed');
			});
  connection.setOnError(function (e) {
			  log('some error!');
			});
  connection.setOnOpen(
      function (e) {
	  log('connected to ROS');
	  connection.callService('/rosbridge/topics', '[]',
	  			 function(rsp){
	  			     log('debug');
	  			     if (checkTopic(rsp, image_topic)) {
	  				 initializeCameraCallback();
	  			     } else {
	  				 log('camera not found');
	  			     }
	  			     if (checkTopic(rsp, '/pc/battery_rate')) {
	  				 initializeBattery('pc', '/pc/battery_rate');
	  			     } else {
	  				 log('pc battery not found');
	  			     }
	  			     if (checkTopic(rsp, '/roomba/battery_charge')) {
	  				 initializeBattery('roomba', '/roomba/battery_charge');
	  			     } else {
	  				 log('roomba battery not found');
	  			     }
	  			     if (checkTopic(rsp, '/arm/battery_rate')) {
	  				 initializeBattery('arm', '/arm/battery_rate');     
	  			     } else {
	  				 log('arm battery not found');
	  			     }
	  			 });
	  // activate roomba
	  connection.callService('/activate', '[]', function(){});
	log('activated roomba');
	// publish for advertise
	connection.publish('/cmd_vel', 'geometry_msgs/Twist', twistMsg(0,0));
	connection.publish('/motion', 'std_msgs/String', stringMsg(""));
	connection.publish('/tray_angle', 'std_msgs/Float64', float64Msg(0));
	connection.publish('/talk', 'std_msgs/String', stringMsg(""));
	connection.publish('/head_angle', 'std_msgs/Float64', float64Msg(0));
      });
}
