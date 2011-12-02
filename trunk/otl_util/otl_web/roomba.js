var connection = null;
var log_canvas = null;

var image_topic = '/usb_cam/image_raw';


function updateImage(vel_x, vel_z) {
  if ( vel_x < 0.0 ) {
    document.upper.src="img/upper2.png";
    document.lower.src="img/lower.png";
  } else if ( vel_x > 0.0 ) {
    document.upper.src="img/upper.png";
    document.lower.src="img/lower2.png";
  } else {
    document.upper.src="img/upper.png";
    document.lower.src="img/lower.png";
  }
  
  if ( vel_z > 0.0 ) {
    document.left.src="img/left2.png";
    document.right.src="img/right.png";
  } else if ( vel_z < 0.0) {
    document.left.src="img/left.png";
    document.right.src="img/right2.png";
  } else {
    document.left.src="img/left.png";
    document.right.src="img/right.png";
  }
  if ( vel_x == 0.0 && vel_z == 0.0 ) {
    document.stop.src="img/stop2.png";
  } else {
    document.stop.src="img/stop.png";
  }
}

function setVelocity(vel_x, vel_z) {
  function twistMsg(x, z) {
    return '{"linear":{"x":' + x + ',"y":0,"z":0}, "angular":{"x":0,"y":0,"z":' + z + '}}';
  }
  updateImage(vel_x, vel_z);
  document.velocity_display.vel.value = 'vel_x: ' + vel_x + '  vel_z: ' + vel_z;
  connection.publish('/cmd_vel', 'geometry_msgs/Twist', twistMsg(vel_x, vel_z));
  log('published x:' + vel_x + ' z:' + vel_z);

}

function boolMsg(data) {
  return '{"data":' + data + '}';
}

function returnToDock() {
  connection.publish('/roomba/dock', 'std_msgs/Bool', boolMsg(true));
  log('published return!');
}

function stringMsg(data) {
  return '{"data":"' + data + '"}';
}

function doCommand(motion_name) {
  connection.publish('/motion', 'std_msgs/String', stringMsg(motion_name));
  log('published: ' + motion_name);
}

function float64Msg(data) {
  return '{"data":' + data + '.0}';
}

function setHeadYawAngle(angle) {
  if (angle > -61 && angle < 61) {
    connection.publish('/tray_angle', 'std_msgs/Float64', float64Msg(angle));
    document.getElementById("range").innerHTML = angle;
    log('published head: ' + angle);
  }
}

function addHeadYawAngle(angle) {
  new_angle = parseFloat(document.getElementById('tray_slider').value) + angle;
  document.getElementById('tray_slider').value = new_angle;
  setHeadYawAngle(new_angle);
}

function talkEnter(event, text) {
  function stringMsg(data) {
    return '{"data":"' + data + '"}';
  }
  var keyCode = event.keyCode ? event.keyCode : event.which ? event.which : event.charCode;
  if (keyCode == 13) {
    log('hoge');
    connection.publish('/talk', 'std_msgs/String', stringMsg(text));
    document.getElementById('talk_input').value = "";
    log('talk: ' + text);
    return false;
  }
  return true;
}

function setHeadPitchAngle(angle) {
  function float64Msg(data) {
    return '{"data":' + data + '.0}';
  }
  if (angle > -61 && angle < 61) {
    connection.publish('/head_angle', 'std_msgs/Float64', float64Msg(angle));
    document.getElementById("head_pitch_value").innerHTML = angle;
    log('published head: ' + angle);
  }
}

function addHeadPitchAngle(angle) {
  new_angle = parseFloat(document.getElementById('head_pitch_slider').value);
  new_angle += angle;
  document.getElementById('head_pitch_slider').value = new_angle;
  setHeadPitchAngle(new_angle);
}
/*
 function LogCanvas(id) {
 function waitForDOM(id) {
 var cnsl = document.getElementById(id);
 if (cnsl == null) {
 setTimeout(waitForDOM(id), 100);
 } else {
 return cnsl;
 }
 }
 this.console = waitForDOM(id);
 this.write = function(msg) {
 this.console.innerHTML = this.console.innerHTML + msg + "<br/>";
 }
 }
 */

function log(msg) {
  document.log.textarea.value = msg + "\n" + document.log.textarea.value;
}

function connectToROS(domain, local_address) {
  function isLocal() {
    var connectInfo = document.location.toString();
    log('url: ' + connectInfo);
    var local_string = connectInfo.search(/^http:\/\/192\.168\./);
    if ( local_string == -1) {
      local_string = connectInfo.search(/^http:\/\/localhost/);
    }
    var file_string = connectInfo.search(/^file:\/\//);
    return (local_string != -1 || file_string != -1);
  }

  var target_host = domain;
  if ( isLocal() ) {
    target_host = local_address;
  }

  var target_uri;
  try {
    target_uri = "ws://" + target_host + ":9090";
    connection = new ros.Connection(target_uri);
  } catch (error) {
    log('error in connection to ' + target_uri);
  }
  log ('rosbridge uri: ' + target_uri);
}

function initializeCameraCallback() {
  try {
    connection.addHandler(image_topic,
			  function(msg) {
			    document.camera.src=msg.uri;
			  });
  } catch (error) {
    log('error in addHandler');
    return;
  }
  try {
    connection.callService('/rosbridge/subscribe', '["' + image_topic + '", 0]', function(rsp){});
    log('camera is connected');
  } catch (error) {
    log('error in subscribe camera image!');
  }
}

function round(num, n) {
  var tmp = Math.pow(10, n);
  return Math.round(num * tmp) / tmp;
}

function initializeBattery(name, topic) {
  try {
    connection.addHandler(topic,
			  function(msg) {
			      document.getElementById(name + '_battery').innerHTML = round(msg.data * 100, 1) + "%";
			      if (msg.data >= 0.8) {
				  document.getElementById(name + '_battery_img').innerHTML = '<img src="img/battery4.png" height="40" width="40"/>';
			      } else if (msg.data >= 0.6) {
				  document.getElementById(name + '_battery_img').innerHTML = '<img src="img/battery3.png" height="40" width="40"/>';
			      } else if (msg.data >= 0.4) {
				  document.getElementById(name + '_battery_img').innerHTML = '<img src="img/battery2.png" height="40" width="40"/>';
			      } else if (msg.data >= 0.2) {
				  document.getElementById(name + '_battery_img').innerHTML = '<img src="img/battery1.png" height="40" width="40"/>';
			      } else {
				  document.getElementById(name + '_battery_img').innerHTML = '<img src="img/battery0.png" height="40" width="40"/>';
			      }
			  });
  } catch (error) {
    log('error in addHandler');
    return;
  }
  try {
    connection.callService('/rosbridge/subscribe', '["' + topic + '", 0]', function(rsp){});
    log(name + ' battery is connected');
  } catch (error) {
    log('error in subscribe' + name + ' battery!');
  }
}

function checkTopic (topics, target) {
  var found = false;
  for (i in topics) {
    if ( topics[i] == target) {
      return true;
    }
  }
  return false;
}

Connection.prototype.delHandler = function(topic) {
  this.handlers[topic] = function(){};
}

  
function initializeSlider(text_id,
			  slider_id,
			  topic) {
  connection.addHandler(topic,
			function(msg) {
			  document.getElementById(text_id).innerHTML = msg.data;
			  document.getElementById(slider_id).value = msg.data;
			  // １回でおわり
			  connection.delHandler(topic);
			});
  try {
    connection.callService('/rosbridge/subscribe', '["' + topic + '", 0]', function(rsp){});
    log(text_id + ' is connected');
  } catch (error) {
    log('error in subscribe' + topic);
  }
}
