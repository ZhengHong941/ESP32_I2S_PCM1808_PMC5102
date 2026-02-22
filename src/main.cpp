#include <Arduino.h>
// #include "driver/i2s.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
// #include "driver/dac_cosine.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "driver/spi_slave.h"

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

#define AUDIO_BUFF_SIZE       4096*2            // Audio buffer size
#define FREQUENCY 48000
#define RECORD_TIME 10000 // in miliseconds

#define I2S_STD_MCLK_IO1        GPIO_NUM_0    // I2S master clock io number
#define I2S_STD_BCLK_IO1        GPIO_NUM_2      // I2S bit clock io number
#define I2S_STD_WS_IO1          GPIO_NUM_15      // I2S word select io number
#define I2S_STD_DOUT_IO1        GPIO_NUM_4      // I2S data out io number
#define I2S_STD_DIN_IO1         GPIO_NUM_16      // I2S data in io number

#define HSPI_SCK 12
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define SD_CS 10

#define RPI_SPI_MOSI  7
#define RPI_SPI_MISO  6
#define RPI_SPI_SCLK  5
#define RPI_SPI_CS    8

#define RECORD_TRIGGER_PIN 9   // Change to your desired GPIO

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

// void i2s_setup();
// static void i2s_record_task(void* args);

void i2s_setup() {
  /* Allocate a pair of I2S channel */
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  /* Allocate for TX and RX channel at the same time, then they will work in full-duplex mode */
  Serial.printf("Init I2S Channel: %x\n", i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  /* Set the configurations for BOTH TWO channels, since TX and RX channel have to be same in full-duplex mode */
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(FREQUENCY),
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

// static void i2s_record_task(void* args)
// {
//   const char* filename = "/record.wav";

//   File audioFile = SD.open(filename, FILE_WRITE);
//   if (!audioFile) {
//     Serial.println("Failed to open WAV file");
//     vTaskDelete(NULL);
//     return;
//   }

//   // Reserve space for WAV header
//   uint8_t header[44] = {0};
//   audioFile.write(header, 44);

//   uint8_t* buffer = (uint8_t*)malloc(AUDIO_BUFF_SIZE);
//   size_t bytesRead = 0;
//   uint32_t totalBytes = 0;

//   Serial.println("Recording...");

//   uint32_t startTime = millis();
//   uint32_t recordTimeMs = RECORD_TIME; // record time

//   // while (millis() - startTime < recordTimeMs) {

//   //   if (i2s_channel_read(rx_handle, buffer, AUDIO_BUFF_SIZE,
//   //                        &bytesRead, portMAX_DELAY) == ESP_OK)
//   //   {
//   //     audioFile.write(buffer, bytesRead);
//   //     totalBytes += bytesRead;
//   //   }
//   // }
//   while (millis() - startTime < recordTimeMs) {

//     if (i2s_channel_read(rx_handle, buffer, AUDIO_BUFF_SIZE,
//                         &bytesRead, portMAX_DELAY) == ESP_OK)
//     {
//       // Save to SD
//       audioFile.write(buffer, bytesRead);
//       totalBytes += bytesRead;

//       // Send to RPi via SPI
//       spi_slave_transaction_t t;
//       memset(&t, 0, sizeof(t));

//       t.length = bytesRead * 8;   // bits
//       t.tx_buffer = buffer;
//       t.rx_buffer = NULL;

//       esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

//       if (ret != ESP_OK) {
//         Serial.println("SPI transmit failed");
//       }
//     }
//   }
//   // after loop ends
//   writeWavHeader(audioFile, totalBytes, FREQUENCY, 32, 2);
//   audioFile.flush();
//   audioFile.close();
//   Serial.printf("WAV done. bytes=%lu\n", (unsigned long)totalBytes);


//   Serial.println("Recording finished");

//   // Write correct WAV header
//   writeWavHeader(audioFile,
//                  totalBytes,
//                  FREQUENCY,
//                  32,
//                  2);

//   audioFile.close();
//   free(buffer);

//   Serial.println("File saved.");
//   vTaskDelete(NULL);
// }

static void i2s_record_task(void* args)
{
  uint8_t* buffer = (uint8_t*)malloc(AUDIO_BUFF_SIZE);
  size_t bytesRead = 0;

  while (true)
  {
    // Wait for trigger HIGH
    Serial.println("Waiting for trigger HIGH...");
    while (digitalRead(RECORD_TRIGGER_PIN) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("Recording started");

    const char* filename = "/record.wav";
    File audioFile = SD.open(filename, FILE_WRITE);

    if (!audioFile) {
      Serial.println("Failed to open WAV file");
      continue;
    }

    // Reserve header space
    uint8_t header[44] = {0};
    audioFile.write(header, 44);

    uint32_t totalBytes = 0;

    // Record while pin stays HIGH
    while (digitalRead(RECORD_TRIGGER_PIN) == HIGH)
    {
      if (i2s_channel_read(rx_handle, buffer, AUDIO_BUFF_SIZE,
                           &bytesRead, portMAX_DELAY) == ESP_OK)
      {
        // Save to SD
        audioFile.write(buffer, bytesRead);
        totalBytes += bytesRead;

        // Send to RPi via SPI
        spi_slave_transaction_t t;
        memset(&t, 0, sizeof(t));

        t.length = bytesRead * 8;   // bits
        t.tx_buffer = buffer;
        t.rx_buffer = NULL;

        spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
      }
    }

    Serial.println("Recording stopped");

    // Write correct WAV header
    writeWavHeader(audioFile, totalBytes, FREQUENCY, 32, 2);

    audioFile.flush();
    audioFile.close();

    Serial.printf("Saved %lu bytes\n", (unsigned long)totalBytes);

    // Wait until trigger fully LOW before rearming
    while (digitalRead(RECORD_TRIGGER_PIN) == HIGH) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("Ready for next trigger");
  }

  free(buffer);
}

void spi_slave_setup()
{
  spi_bus_config_t buscfg = {
    .mosi_io_num = RPI_SPI_MOSI,
    .miso_io_num = RPI_SPI_MISO,
    .sclk_io_num = RPI_SPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = AUDIO_BUFF_SIZE
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num = RPI_SPI_CS,
    .flags = 0,
    .queue_size = 3,
    .mode = 0,
    .post_setup_cb = NULL,
    .post_trans_cb = NULL
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  Serial.printf("SPI Slave Init: %d\n", ret);
}

void setup() {

  // Set up Serial Monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("hello");
  pinMode(RECORD_TRIGGER_PIN, INPUT);

  delay(2000);
  // setup SD card reader
  hspi = new SPIClass(HSPI);                           // create SPI class
  hspi->begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SD_CS);  // setup SPI pins
  if (!SD.begin(SD_CS, *hspi)) {                       // mount card
    Serial.println("Card Mount Failed");
    // return;
  }

  // Set up I2S
  i2s_setup();
  spi_slave_setup();
  
  delay(500);

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