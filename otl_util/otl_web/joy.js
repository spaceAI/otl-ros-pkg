var canvas;
var ctx;
var timerDrawID;  //描画タイマーID
var cW = 150;  //キャンバス横サイズ
var cH = 150;  //キャンバス縦サイズ
var mouseDownFlag = false;  //マウスダウンしているかどうか
var mouseX1;  //ドラッグ開始した座標
var mouseY1;  //ドラッグ開始した座標
var mouseX2;  //ドラッグ中の座標
var mouseY2;  //ドラッグ中の座標
var dragDivX;  //ドラッグ開始地点と現在の差
var dragDivY;  //ドラッグ開始地点と現在の差
var obj;
//ユーザーエージェント
var isiPad;
var isiPhone;

function joy_init(){
  //キャンバスの初期処理
  canvas = document.getElementById('joyCanvas');
  if ( ! canvas || ! canvas.getContext ) return false;
  //2Dコンテキスト
  ctx = canvas.getContext('2d');
  //イベント
  canvas.addEventListener("mousedown", mouseDownListner, false);
  canvas.addEventListener("mousemove", mouseMoveListner, false);
  canvas.addEventListener("mouseup", mouseUpListner, false);
  canvas.addEventListener("mouseout", mouseUpListner, false);
  canvas.addEventListener("touchstart", mouseDownListner, false);
  canvas.addEventListener("touchmove", mouseMoveListner, false);
  canvas.addEventListener("touchend", mouseUpListner, false);
  canvas.addEventListener("touchcancel", mouseUpListner, false);
  //オブジェクト
  obj = new Object();
  obj.w = 40;
  obj.h = 40;
  obj.x = cW*0.5 - obj.w * 0.5;
  obj.y = cH*0.5 - obj.h * 0.5;

  //ユーザーエージェント
  isiPad = navigator.userAgent.match(/iPad/i) != null;
  isiPhone = navigator.userAgent.match(/iPhone/i) != null;

  //タイマー開始
  setTimerDraw();
}

function setTimerDraw(){
  clearInterval(timerDrawID);
  timerDrawID = setInterval("joy_draw()", 100);
}

function joy_draw() {
  //表示クリア
  ctx.clearRect(0, 0, cW, cH);
  ctx.fillStyle="#000000";
  ctx.beginPath();

  ctx.rect(10, 10, cW-20, cH-20);
  ctx.rect(30, 30, cW-60, cH-60);
  ctx.rect(50, 50, cW-100, cH-100);

  ctx.moveTo(0,cH*0.5);
  ctx.lineTo(cW,cH*0.5);
  ctx.moveTo(cW*0.5,0);
  ctx.lineTo(cW*0.5,cH);
  ctx.stroke();

  //画像を描く
  if (mouseDownFlag) {
    ctx.fillStyle="#FF0000";
    ctx.fillRect(obj.x + dragDivX, obj.y + dragDivY, obj.w, obj.h);
  } else {
    ctx.fillRect(obj.x, obj.y, obj.w, obj.h);
  }
}

function twistMsg(x, z) {
  return '{"linear":{"x":' + x + ',"y":0,"z":0}, "angular":{"x":0,"y":0,"z":' + z + '}}';
}

//マウスイベント
function mouseDownListner(e) {
  var rect = e.target.getBoundingClientRect();
  //座標取得
  if (isiPad || isiPhone) {
    //iPad & iPhone用処理
    mouseX1 = e.touches[0].pageX - rect.left;
    mouseY1 = e.touches[0].pageY - rect.top;
  } else {
    //PC用処理
    mouseX1 = e.clientX - rect.left;
    mouseY1 = e.clientY - rect.top;
  }
  if (mouseX1 > obj.x && mouseX1 < obj.x + obj.w) {
    if (mouseY1 > obj.y && mouseY1 < obj.y + obj.h) {
      dragDivX = 0;
      dragDivY = 0;
      mouseDownFlag = true;
    }
  }
}

function mouseMoveListner(e) {
  e.preventDefault();
  if (mouseDownFlag) {
    //縦スクロールをしない（iPad & iPhone）
    //座標取得
    var rect = rect = e.target.getBoundingClientRect();
    if (isiPad || isiPhone) {
      //iPad & iPhone用処理
      mouseX2 = e.touches[0].pageX - rect.left;
      mouseY2 = e.touches[0].pageY - rect.top;
    } else {
      //PC用処理
      mouseX2 = e.clientX - rect.left;
      mouseY2 = e.clientY - rect.top;
    }
    if (mouseX2 < 0 || mouseX2 > cW || mouseY2 < 0 || mouseY2 > cH) {
      dragEnd();
    }
    dragDivX = (mouseX2 - mouseX1);
    dragDivY = (mouseY2 - mouseY1);
    // ROS
    connection.publish('/cmd_vel', 'geometry_msgs/Twist', twistMsg(dragDivY * 0.001, dragDivX * -0.003));
  }
}

function mouseUpListner(e) {
  dragEnd();
}

function dragEnd() {
  mouseDownFlag = false;
  obj.x = cW*0.5 - obj.w*0.5;
  obj.y = cH*0.5 - obj.h*0.5;
  connection.publish('/cmd_vel', 'geometry_msgs/Twist', twistMsg(0, 0));
  connection.publish('/cmd_vel', 'geometry_msgs/Twist', twistMsg(0, 0));

}
