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
			     });
      // activate roomba
      connection.callService('/activate', '[]', function(){});
      log('activated roomba')
      function handleKey(code, down) {
	var scale = 0;
	if (down == true) {
	  scale = 1;
	}
	switch (code) {
	case 37:
	  //left
	  vel_z = 0.5 * scale;
	  break;
	case 38:
	  //up
	  vel_x = .5 * scale;
	  break;
	case 39:
	  //right
	  vel_z = -0.5 * scale;
	  break;
	case 40:
	  //down
	  vel_x = -.5 * scale;
	  break;
	}
	setVelocity(vel_x, vel_z);
      }

      document.addEventListener('keydown', function (e) {
				  handleKey(e.keyCode, true);
				}, true);
      document.addEventListener('keyup', function (e) {
				  handleKey(e.keyCode, false);
				}, true);
    });
}
