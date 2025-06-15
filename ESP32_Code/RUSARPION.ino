#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include <WiFi.h>
//#include <WiFiManager.h>

#include "cobs.h"
//#define DebugSerial

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_GPIO_NUM      33

//AUDIO
//#include <HardwareSerial.h>
//#include "DFRobotDFPlayerMini.h"

//HardwareSerial DFPlayerSerial(1);

//DFRobotDFPlayerMini DFPlayer;

//
static esp_err_t parse_get(httpd_req_t *req, char **obuf)
{
    char *buf = NULL;
    size_t buf_len = 0;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            *obuf = buf;
            return ESP_OK;
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#include "html_index.h"

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128];

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    while (true)
    {

        fb = esp_camera_fb_get();
        if (!fb)
        {
            log_e("Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            _timestamp.tv_sec = fb->timestamp.tv_sec;
            _timestamp.tv_usec = fb->timestamp.tv_usec;
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            log_e("Send frame failed");
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t frame_time = fr_end - last_frame;
        frame_time /= 1000;
    }
    return res;
}

String HEX_convert(uint8_t* uid, int size) {
  String hexString = "";
  for (int i = 0; i < size; i++) {
    if (uid[i] < 0x10) {
      hexString += '0'; 
    }
    hexString += String(uid[i], HEX);  
  }
  hexString.toUpperCase();
  return hexString;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
  //GET POSS
  char *buf = NULL;
  char req_moveX[8];
  char req_moveY[8];
  char req_rotP[8];
  char req_speed[8];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "move_x", req_moveX, sizeof(req_moveX)) != ESP_OK ||
      httpd_query_key_value(buf, "move_y", req_moveY, sizeof(req_moveY)) != ESP_OK ||
      httpd_query_key_value(buf, "rot_pow", req_rotP, sizeof(req_rotP)) != ESP_OK ||
      httpd_query_key_value(buf, "speed", req_speed, sizeof(req_speed)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  uint8_t _moveX = atoi(req_moveX);
  uint8_t _moveY = atoi(req_moveY);
  uint8_t _rotP  = atoi(req_rotP);
  uint8_t _speed = atoi(req_speed);

  //DATA
  uint8_t _data[4];

  _data[0] = _moveX;
  _data[1] = _moveY;
  _data[2] = _rotP;
  _data[3] = _speed;

  //CRC AND COBS
  uint8_t crc_data[6];
  
  for(int i = 0; i < 4; i++){
    crc_data[i] = _data[i];
  }
  uint16_t crc = CalculateCrc(_data, 4);
  crc_data[4] = (crc & 0xff00) >> 8;
  crc_data[5] = crc & 0x00ff;
  
  uint8_t cobs_data[8];
  CobsEncode(cobs_data, crc_data, 6);
  
#ifdef DebugSerial
  Serial.print("data: ");
  Serial.println(HEX_convert(cobs_data, 8));
#else
  Serial.write(cobs_data, 8);
#endif

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t audio_handler(httpd_req_t *req)
{
  //GET POSS
  char *buf = NULL;
  
  char req_audio[8];
  char req_volume[8];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "audio", req_audio, sizeof(req_audio)) != ESP_OK ||
      httpd_query_key_value(buf, "volume", req_volume, sizeof(req_volume)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  //AUDIO DATA
  uint8_t _audio = atoi(req_audio);
  uint8_t _volume = atoi(req_volume);
  /*
  if (_audio != 0){
    DFPlayer.play(_audio);
  }
  if(_volume > 30) _volume = 30;
  DFPlayer.volume(_volume);*/

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

//________START_FUNC________
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 3;
  
  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
      ,
      .is_websocket = true,
      .handle_ws_control_frames = false,
      .supported_subprotocol = NULL
#endif
  };
    
  httpd_uri_t cmd_uri = {
      .uri = "/control",
      .method = HTTP_GET,
      .handler = cmd_handler,
      .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
      ,
      .is_websocket = true,
      .handle_ws_control_frames = false,
      .supported_subprotocol = NULL
#endif
  };
    
  httpd_uri_t audio_uri = {
      .uri = "/audio",
      .method = HTTP_GET,
      .handler = audio_handler,
      .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
      ,
      .is_websocket = true,
      .handle_ws_control_frames = false,
      .supported_subprotocol = NULL
#endif
  };
  
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &audio_uri);
  }
  
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
  };
  
  config.server_port += 1;
  config.ctrl_port += 1;
#ifdef DebugSerial
  Serial.print("Starting stream server on port: ");
  Serial.println(config.server_port);
#endif
  
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
//___________________________________


const char* ssid = "robots";//"ForESP-2";//"HabLab-Scena-2";
const char* password = "";//"11111111";//"hablabksu";

IPAddress staticIP(192, 168, 0, 100); // Желаемый статический IP-адрес
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
 /* DFPlayerSerial.begin(9600, SERIAL_8N1, 2, 14);
  
  if (!DFPlayer.begin(DFPlayerSerial)) {  //Use softwareSerial to communicate with mp3.
#ifdef DebugSerial
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
#endif
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }*/
  
 // DFPlayer.volume(30);  //Set volume value. From 0 to 30
  
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);

  // Установка статического IP-адреса
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);
  
  //WiFi.softAP("RUSARPION"); // без пароля
  
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef DebugSerial
    Serial.print(".");
#endif
  }
#ifdef DebugSerial
  Serial.println("");
  Serial.println("WiFi connected");
#endif

  startCameraServer();
#ifdef DebugSerial
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
#endif
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  
  if(psramFound()){
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
#ifdef DebugSerial
    Serial.printf("Camera init failed with error 0x%x", err);
#endif
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);
  s->set_brightness(s, 1.5);
  s->set_aec_value(s, 600);
  s->set_saturation(s, 1);
  s->set_vflip(s, 1);
  
  /*WiFiManager wm;
  bool res = wm.autoConnect("RUSARPION","");
  Serial.println("Start WiFiManager RUSARPION");
#ifdef DebugSerial
  if(!res) Serial.println("Failed to connect");
  else Serial.println("connected...yeey :)");
#endif*/
  
  
}

void loop() {
  delay(10000);
}
