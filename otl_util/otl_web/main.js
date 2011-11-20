function main() {
//  connectToROS("roomba.dyndns.tv", "192.168.11.4");
//  connectToROS("mtm07otl.ddo.jp", "192.168.100.101");
  joy_init();
  connectToROS("mtm07otl.ddo.jp", "192.168.11.4");
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
      connection.callService('/rosjs/topics', '[]',
			     function(rsp){
			       if (checkTopic(rsp, image_topic)) {
				 initializeCameraCallback();
			       } else {
				 log('camera not found');
			       }
			       if (checkTopic(rsp, '/pc/battery_rate')) {
				 initializeBattery('pc', '/pc/battery_rate');
			       } else {
				 log('pc battery not found')
			       }
			       if (checkTopic(rsp, '/roomba/battery_charge')) {
				 initializeBattery('roomba', '/roomba/battery_charge');
			       } else {
				 log('roomba battery not found')
			       }
			       if (checkTopic(rsp, '/arm/battery_rate')) {
				 initializeBattery('arm', '/arm/battery_rate');
			       } else {
				 log('arm battery not found')
			       }
			     });
      // activate roomba
      connection.callService('/activate', '[]', function(){});
      log('activated roomba');
    });
}
