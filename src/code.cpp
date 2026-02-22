#include <Arduino.h>
// #include "driver/i2s.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
// #include "driver/dac_cosine.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

#define AUDIO_BUFF_SIZE       4096*2            // Audio buffer size

#define I2S_STD_MCLK_IO1        GPIO_NUM_0    // I2S master clock io number
#define I2S_STD_BCLK_IO1        GPIO_NUM_2      // I2S bit clock io number
#define I2S_STD_WS_IO1          GPIO_NUM_15      // I2S word select io number
#define I2S_STD_DOUT_IO1        GPIO_NUM_4      // I2S data out io number
#define I2S_STD_DIN_IO1         GPIO_NUM_16      // I2S data in io number

#define HSPI_SCK 12
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define SD_CS 10

SPIClass *hspi = NULL;  // SPI object

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void writeWavHeader(File &file, uint32_t dataSize, uint32_t sampleRate,
                    uint16_t bitsPerSample, uint16_t channels)
{
  uint32_t byteRate = sampleRate * channels * bitsPerSample / 8;
  uint16_t blockAlign = channels * bitsPerSample / 8;
  uint32_t chunkSize = 36 + dataSize;

  file.seek(0);

  file.write((const uint8_t *)"RIFF", 4);
  file.write((uint8_t *)&chunkSize, 4);
  file.write((const uint8_t *)"WAVE", 4);

  file.write((const uint8_t *)"fmt ", 4);
  uint32_t subChunk1Size = 16;
  uint16_t audioFormat = 1;

  file.write((uint8_t *)&subChunk1Size, 4);
  file.write((uint8_t *)&audioFormat, 2);
  file.write((uint8_t *)&channels, 2);
  file.write((uint8_t *)&sampleRate, 4);
  file.write((uint8_t *)&byteRate, 4);
  file.write((uint8_t *)&blockAlign, 2);
  file.write((uint8_t *)&bitsPerSample, 2);

  file.write((const uint8_t *)"data", 4);
  file.write((uint8_t *)&dataSize, 4);
}

void i2s_setup();
// void dac_setup();
static void i2s_example_read_task(void* args);
static void i2s_example_write_task(void* args);
// static void i2s_example_dsp_task(void* args);
static void i2s_record_task(void* args);

void i2s_setup() {
  /* Allocate a pair of I2S channel */
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  /* Allocate for TX and RX channel at the same time, then they will work in full-duplex mode */
  Serial.printf("Init I2S Channel: %x\n", i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  /* Set the configurations for BOTH TWO channels, since TX and RX channel have to be same in full-duplex mode */
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_STD_MCLK_IO1,
          .bclk = I2S_STD_BCLK_IO1,
          .ws = I2S_STD_WS_IO1,
          .dout = I2S_STD_DOUT_IO1,
          .din = I2S_STD_DIN_IO1,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
      },
  };
  std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  Serial.printf("Init I2S TX: %x\n", i2s_channel_init_std_mode(tx_handle, &std_cfg));
  Serial.printf("Init I2S RX: %x\n", i2s_channel_init_std_mode(rx_handle, &std_cfg));

  Serial.printf("Enable I2S TX: %x\n", i2s_channel_enable(tx_handle));
  Serial.printf("Enable I2S RX: %x\n", i2s_channel_enable(rx_handle));
}


static void i2s_record_task(void* args)
{
  const char* filename = "/record.wav";

  File audioFile = SD.open(filename, FILE_WRITE);
  if (!audioFile) {
    Serial.println("Failed to open WAV file");
    vTaskDelete(NULL);
    return;
  }

  // Reserve space for WAV header
  uint8_t header[44] = {0};
  audioFile.write(header, 44);

  uint8_t* buffer = (uint8_t*)malloc(AUDIO_BUFF_SIZE);
  size_t bytesRead = 0;
  uint32_t totalBytes = 0;

  Serial.println("Recording...");

  uint32_t startTime = millis();
  uint32_t recordTimeMs = 10000; // 10 seconds

  while (millis() - startTime < recordTimeMs) {

    if (i2s_channel_read(rx_handle, buffer, AUDIO_BUFF_SIZE,
                         &bytesRead, portMAX_DELAY) == ESP_OK)
    {
      audioFile.write(buffer, bytesRead);
      totalBytes += bytesRead;
    }
  }

  Serial.println("Recording finished");

  // Write correct WAV header
  writeWavHeader(audioFile,
                 totalBytes,
                 48000,
                 32,
                 2);

  audioFile.close();
  free(buffer);

  Serial.println("File saved.");
  vTaskDelete(NULL);
}


void setup() {

  // Set up Serial Monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("hello");

  delay(2000);
  // setup SD card reader
  hspi = new SPIClass(HSPI);                           // create SPI class
  hspi->begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SD_CS);  // setup SPI pins
  if (!SD.begin(SD_CS, *hspi)) {                       // mount card
    Serial.println("Card Mount Failed");
    return;
  }


  // Set up I2S
  i2s_setup();

  //Set up dac to generate 440Hz test tone
  // dac_setup();

  delay(500);

  // writeFile(SD, "/hello.txt", "Hello ");

  // Loopback/DSP task
  // xTaskCreate(i2s_example_dsp_task, "i2s_example_dsp_task", 4096, NULL, 5, NULL);

  // Debug tasks
  // xTaskCreate(i2s_example_read_task, "i2s_example_read_task", 4096, NULL, 5, NULL);
  //xTaskCreate(i2s_example_write_task, "i2s_example_write_task", 4096, NULL, 5, NULL);

  // Start recording task
  xTaskCreate(i2s_record_task,
              "i2s_record_task",
              8192,
              NULL,
              5,
              NULL);
}

void loop() {
  delay(1000);
}

// Modified read example
static void i2s_example_read_task(void* args)
{
  uint8_t* r_buf = (uint8_t*)calloc(1, AUDIO_BUFF_SIZE);
  assert(r_buf); // Check if r_buf allocation success
  size_t r_bytes = 0;

  /* Enable the RX channel */
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

  /* Print might make this loop way too slow, consider limiting amount of info printed and lowering the sample rate */
  while (1) {
    /* Read i2s data */
    if (i2s_channel_read(rx_handle, r_buf, AUDIO_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) 
    {
      Serial.printf("%" PRId32  "\n", (r_buf[0] << 24) | (r_buf[1] << 16) | (r_buf[2] << 8) | (r_buf[3]));
    }
    else {
      Serial.printf("Read Task: i2s read failed\n");
    }
    vTaskDelay(pdMS_TO_TICKS(16));
  }
  free(r_buf);
  vTaskDelete(NULL);
}

// static void i2s_example_dsp_task(void* args)
// {
//   size_t w_bytes = AUDIO_BUFF_SIZE;

//   /* Enable the TX channel */
//   ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

//   uint8_t* r_buf = (uint8_t*)calloc(1, AUDIO_BUFF_SIZE);
//   assert(r_buf); // Check if r_buf allocation success
//   size_t r_bytes = 0;

//   /* Enable the RX channel */
//   ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

//   Serial.println("Ready for write.");

//   while (1) {
//     /* Read i2s data */
//     auto err_r = i2s_channel_read(rx_handle, r_buf, AUDIO_BUFF_SIZE, &r_bytes, 1000);
//     if (err_r != ESP_OK) {
//       Serial.printf("DSP Task: i2s read failed with code: %d\n", err_r);
//     }

//     // TODO: Add audio processing here

//     /* Write i2s data */
//     auto err_w = i2s_channel_write(tx_handle, r_buf, r_bytes, &w_bytes, 1000);
//     if (err_w != ESP_OK) {
//       Serial.printf("DSP Task: i2s write failed with code: %d\n", err_w);
//     }

//     vTaskDelay(pdMS_TO_TICKS(16));
//   }

//   free(r_buf);
//   vTaskDelete(NULL);
// }