#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

static const char PROGMEM INDEX_HTML[] = R"rawliteral(<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width,initial-scale=1">
        <title>ESP32 CAM Robot</title>
<style>
*{
  font-family: Arial;
}
div {
  text-align: center;
  margin-top: 20px;
}

table {
  table-layout: fixed;
  margin: auto;
  width: 20%;
}

.slider-container td{
  text-align:right;
}
button {
  border:none;
    border-radius:5px;
    background:#31d4ce;
  
  width:100%;
  padding:20%;
}

button:hover{
    background:#37eb34;
}

button:active{
    background:#9bff99;
}

.tr-settings button{
  padding:5%;
}
#stream{
  width:30%;
}

</style>

<style  media="screen">

div {
  text-align: left;
}

table {
  margin-left: 0;
  width:100%;
}

.image-container{
  //position:absolute;
  right:0%;
  width:100%;
}
#stream{
  width:100%;
  margin:auto;
}
.div_buttons{
  width:100%;
  margin:auto;
}
.div_buttons table{
  width:50%;
  margin:auto;
}
</style>

<style  media="handheld">

table {
  width: 100%;
}
#stream{
  width:100%;
}
</style>

    </head>
    <body>
  <table>
  <tr><td>
    <div id="stream-container" class="image-container">
      <img id="stream" src="img2.jpg">
    </div>
    </td>
    <td>
    <div id="controls" class="div_buttons">
      <table>
        <tr class="tr-settings">
          <td>
          </td>
          <td>
            <button id="toggle-stream">Почати</button>
          </td>
          <td></td>
        </tr>
        <tr>
          <td style="text-align:right;">
            <button id="turnleftrotate">ОБЕРТ+</button>
          </td>
          <td>
            <button id="forward">ВПЕРЕД</button>
          </td>
          <td style="text-align:left;">
            <button id="turnrightrotate">ОБЕРТ-</button>
          </td>
        </tr>
        <tr>
          <td style="text-align:right;">
            <button id="turnleft">ВЛІВО</button>
          </td>
          <td>
            <button id="stopbut">СТОП</button>
          </td>
          <td style="text-align:left;">
            <button id="turnright">ВПРАВО</button>
          </td>
        </tr>
        <tr>
          <td></td>
          <td>
            <button id="backward">НАЗАД</button>
          </td>
          <td></td>
        </tr>
      </table>
    </div>
    </td>
    </tr>
    </table>
    <script>
      document.addEventListener("DOMContentLoaded", function() {
       function callback(el) {
         var b = void 0;
         switch(el.type) {
         case "checkbox":
           b = el.checked ? 1 : 0;
           break;
         case "range":
         case "select-one":
           b = el.value;
           break;
         case "button":
         case "submit":
           b = "1";
           break;
         default:
           return;
         }
         var css = url + "/control?var=" + el.id + "&val=" + b;
         fetch(css).then(function(testsStatus) {
         console.log("request to " + css + " finished, status: " + testsStatus.status);
         });
       }
       var url = document.location.origin;
       var cb = function onchange(data, key, name) {
         name = !(null != name) || name;
         var val = void 0;
         if ("checkbox" === data.type) {
         val = data.checked;
         key = !!key;
         data.checked = key;
         } else {
         val = data.value;
         data.value = key;
         }
         if (name && val !== key) {
         callback(data);
         }
       };
       fetch(url + "/status").then(function(rawResp) {
         return rawResp.json();
       }).then(function(data) {
         document.querySelectorAll(".default-action").forEach(function(o) {
         cb(o, data[o.id], false);
         });
       });
       var s1 = document.getElementById("stream");
       var end = document.getElementById("stream-container");
       var button = document.getElementById("toggle-stream");
       var hideCalen = function tick() {
         window.stop();
         button.innerHTML = "Почати";
       };
       var acceptTheCall = function run() {
         s1.src = url + ":81" + "/stream";
         button.innerHTML = "Зупинити";
       };
       button.onclick = function() {
         var B = "Зупинити" === button.innerHTML;
         if (B) {
         hideCalen();
         } else {
         acceptTheCall();
         }
       };
       document.querySelectorAll(".default-action").forEach(function(identifierPositions) {
         identifierPositions.onchange = function() {
         return callback(identifierPositions);
         };
       });
       var w = document.getElementById("framesize");
       w.onchange = function() {
         callback(w);
       };
      });
      
      document.addEventListener('keydown', function(event) {
        if (event.code == 'KeyW') fetch(document.location.origin+'/control?move_x=127&move_y=255&rot_pow=127&speed=120');
        if (event.code == 'KeyS') fetch(document.location.origin+'/control?move_x=127&move_y=0&rot_pow=127&speed=120');
        if (event.code == 'KeyD') fetch(document.location.origin+'/control?move_x=255&move_y=127&rot_pow=127&speed=120');
        if (event.code == 'KeyA') fetch(document.location.origin+'/control?move_x=0&move_y=127&rot_pow=127&speed=120');
        if (event.code == 'KeyE') fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=255&speed=120');
        if (event.code == 'KeyQ') fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=0&speed=120');
      });
      
      document.addEventListener('keyup', function(event) {
        fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=127&speed=0');
      });
      
      forward_but = document.getElementById("forward");
      backward_but = document.getElementById("backward");
      right_but = document.getElementById("turnright");
      left_but = document.getElementById("turnleft");
      rightrotate_but = document.getElementById("turnrightrotate");
      leftrotate_but = document.getElementById("turnleftrotate");
      stop_but = document.getElementById("stopbut");
      
      if (/Android|webOS|iPhone|iPad|iPod|BlackBerry|BB|PlayBook|IEMobile|Windows Phone|Kindle|Silk|Opera Mini/i.test(navigator.userAgent)) {

        stop_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=127&speed=0');
        });
        
        forward_but.addEventListener('touchstart', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=255&rot_pow=127&speed=120');
        });
        forward_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=255&rot_pow=127&speed=120');
        });
        
        backward_but.addEventListener('touchstart', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=0&rot_pow=127&speed=120');
        });
        backward_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=0&rot_pow=127&speed=120');
        });
      
        right_but.addEventListener('touchstart', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=255&move_y=127&rot_pow=127&speed=120');
        });
        right_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=255&move_y=127&rot_pow=127&speed=120');
        });
        
        left_but.addEventListener('touchstart', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=0&move_y=127&rot_pow=127&speed=120');
        });
        left_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=0&move_y=127&rot_pow=127&speed=120');
        });
      
        rightrotate_but.addEventListener('touchstart', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=255&speed=120');
        });
        rightrotate_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=255&speed=120');
        });
        
        leftrotate_but.addEventListener('touchstart', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=0&speed=120');
        });
        leftrotate_but.addEventListener('touchend', (e) => {
          e.preventDefault();
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=0&speed=120');
        });
      }else{
          
        stop_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=127&speed=0');
        });
        
        forward_but.addEventListener('mousedown', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=255&rot_pow=127&speed=120');
        });
        forward_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=255&rot_pow=127&speed=120');
        });
        
        backward_but.addEventListener('mousedown', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=0&rot_pow=127&speed=120');
        });
        backward_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=0&rot_pow=127&speed=120');
        });
        
        right_but.addEventListener('mousedown', (e) => {
          fetch(document.location.origin+'/control?move_x=255&move_y=127&rot_pow=127&speed=120');
        });
        right_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=255&move_y=127&rot_pow=127&speed=120');
        });
        
        left_but.addEventListener('mousedown', (e) => {
          fetch(document.location.origin+'/control?move_x=0&move_y=127&rot_pow=127&speed=120');
        });
        left_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=0&move_y=127&rot_pow=127&speed=120');
        });
        
        rightrotate_but.addEventListener('mousedown', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=255&speed=120');
        });
        rightrotate_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=255&speed=120');
        });
        
        leftrotate_but.addEventListener('mousedown', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=0&speed=120');
        });
        leftrotate_but.addEventListener('mouseup', (e) => {
          fetch(document.location.origin+'/control?move_x=127&move_y=127&rot_pow=0&speed=120');
        });
      }
    </script>
    </body>
</html>
)rawliteral";
