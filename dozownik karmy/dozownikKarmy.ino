#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL49uddtYVH"
#define BLYNK_TEMPLATE_NAME "PetFeeder"
#define BLYNK_AUTH_TOKEN "PDvmZNm3lmIASmma8zCf-O4kfAFxQlYR"

#include <Cat_and_Dog_Recognition_inferencing.h> 
//#include <ESP32_Servo.h> 
#define CAMERA_MODEL_AI_THINKER

#include "img_converters.h"
#include "image_util.h"
#include "esp_camera.h"
#include "camera_pins.h"

char ssid[] = "*****";
char pass[] = "*******";


String cat="Cat";
String dog="Dog";

dl_matrix3du_t *resized_matrix = NULL;
ei_impulse_result_t result = {0};

const int trigPin = 13;
const int echoPin = 15;

int led1=2;//niebieska 
int led2=4; //czerwona

void setup() {
  Serial.begin(115200);
 

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);


  // cam config
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); 
    s->set_brightness(s, 1); 
    s->set_saturation(s, 0);
  }

  Serial.println("Camera Ready!");
    
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
 


  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
}



void loop() {
   Blynk.run();
  String res;
  float dystans = zmierzOdleglosc();
  if(dystans<=20 && dystans >1){
   res = classify();
    if(res==cat){
    digitalWrite(led1,HIGH);//niebieska
    delay(2000);
  }
  else if(res ==dog){
     // display result
    Serial.printf("Result: %s\n", res);
    digitalWrite(led2,HIGH);//czerwona
    delay(2000);
  }
  }
  else{
    res="None";
    digitalWrite(led1,LOW);
    digitalWrite(led2,LOW);
  }


  
 
 

  Serial.print("dystans: ");
  Serial.println(dystans);
 

}

// Klasyfikacja
String classify() {

  
  capture_quick();


  if (!capture()) return "Error";

 
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_WIDTH;
  signal.get_data = &raw_feature_get_data;

  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);

  dl_matrix3du_free(resized_matrix);

  ei_printf("run_classifier returned: %d\n", res);
  if (res != 0) return "Error";

  // --- print the predictions ---
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  int index;
  float score = 0.0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {

    if (result.classification[ix].value > score) {
      score = result.classification[ix].value;
      index = ix;
    }
    ei_printf("    %s: \t%f\r\n", result.classification[ix].label, result.classification[ix].value);
  }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %f\r\n", result.anomaly);
#endif


  return String(result.classification[index].label);
}

void capture_quick() {
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) return;
  esp_camera_fb_return(fb);
}

bool capture() {

  Serial.println("Capture image...");
  esp_err_t res = ESP_OK;
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  Serial.println("Converting to RGB888...");

  dl_matrix3du_t *rgb888_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_matrix->item);

  Serial.println("Resizing the frame buffer...");
  resized_matrix = dl_matrix3du_alloc(1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
  image_resize_linear(resized_matrix->item, rgb888_matrix->item, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3, fb->width, fb->height);

  dl_matrix3du_free(rgb888_matrix);
  esp_camera_fb_return(fb);

  return true;
}

int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr) {

  size_t pixel_ix = offset * 3;
  size_t bytes_left = out_len;
  size_t out_ptr_ix = 0;


  while (bytes_left != 0) {
  
    uint8_t r, g, b;
    r = resized_matrix->item[pixel_ix];
    g = resized_matrix->item[pixel_ix + 1];
    b = resized_matrix->item[pixel_ix + 2];

    float pixel_f = (r << 16) + (g << 8) + b;
    signal_ptr[out_ptr_ix] = pixel_f;

    out_ptr_ix++;
    pixel_ix += 3;
    bytes_left--;
  }

  return 0;
}


	
int zmierzOdleglosc() {
  long czas, dystans;
 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  czas = pulseIn(echoPin, HIGH);
  dystans = czas / 58;
 
  return dystans;
}
