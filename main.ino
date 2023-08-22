/******************************************************************************
  
  HARDWARE CONNECTIONS

  **********************
  ESP32 ------- CODEC
  **********************
  QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
  GND --------- GND         *optional, but not a bad idea
  5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)
  4 ----------- DDT         *aka DAC_DATA/I2S_SDO/"serial data out", this carries the I2S audio data from ESP32 to codec DAC
  16 ---------- BCK         *aka BCLK/I2S_SCK/"bit clock", this is the clock for I2S audio, can be controlled via controller or peripheral.
  17 ---------- ADAT        *aka ADC_DATA/I2S_SD/"serial data in", this carries the I2S audio data from codec's ADC to ESP32 I2S bus.
  25 ---------- DLRC        *aka I2S_WS/LRC/"word select"/"left-right-channel", this toggles for left or right channel data.
  25 ---------- ALR         *for this example WS is shared for both the ADC WS (ALR) and the DAC WS (DLRC)

  **********************
  CODEC ------- AUDIO IN
  **********************
  GND --------- TRS INPUT SLEEVE        *ground for line level input
  LINPUT1 ----- TRS INPUT TIP           *left audio
  RINPUT1 ----- TRS INPUT RING1         *right audio

  **********************
  CODEC -------- AUDIO OUT
  **********************
  OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
  HPL ---------- TRS OUTPUT TIP             *left HP output
  HPR ---------- TRS OUTPUT RING1           *right HP output
******************************************************************************/

#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> 
WM8960 codec;

// Include I2S driver
#include <driver/i2s.h>

// Connections to I2S
#define I2S_WS 25
#define I2S_SD 18 
#define I2S_SDO 4 
#define I2S_SCK 15

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// Define input buffer length
#define bufferLen 64
int16_t sBuffer[bufferLen];

void setup() {
  Serial.begin(115200);

  Wire.begin();

  if (codec.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); // Freeze
  }
  Serial.println("Device is connected properly.");

  codec_setup();

  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
}

//distortion effect cuts off highest highs and lowest lows of the signal. Signal im getting from my guitar with no additional gain spans from ~ -
void distortion(int16_t *buffer, int len) {
  
  //this is the value we will limit our signal to. All samples exceeding threshold will be set to its value.
  const int threshold = 30000;

  for(int i = 0; i < len; i++) {
    if(buffer[i] > threshold) 
      buffer[i] = threshold;
    else if(buffer[i] < -threshold) 
      buffer[i] = -threshold;
    Serial.println(buffer[i]);
  }
}

int16_t *delay_buffer;
int nRepeats = 3;
int delay_time = 100; //100 ms of delay between repeats

//delay makes an "echo" of our signal, repeating what we played after a fixed amount of time, a fixed amount of times
void delay(int16_t *buffer, int len) {


}


int distortion_on = 1;
int delay_on = 0;

void loop()
{
  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  size_t bytesOut = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  
  if(distortion_on == 1) {
    long int t1 = micros();
    //distortion(sBuffer, bufferLen);
    for(int i = 0; i < bufferLen; i++){
      int watch_value = sBuffer[i];
      Serial.println(watch_value);
    }
      
    long int t2 = micros();
    Serial.print("Time taken by the task: "); Serial.print(t2-t1); Serial.println(" microseconds");
    distortion_on = 0;
  }
  

  //Printing to serial takes a lot of time :l

  if (result == ESP_OK)
  {
    // Send what we just received back to the codec
    esp_err_t result_w = i2s_write(I2S_PORT, &sBuffer, bytesIn, &bytesOut, portMAX_DELAY);

    // If there was an I2S write error, let us know on the serial terminal
    if (result_w != ESP_OK)
    {
      Serial.print("I2S write error.");
    }
  }
  // DelayMicroseconds(300); // Only hear to demonstrate how much time you have 
  // to do things.
  // Do not do much in this main loop, or the audio won't pass through correctly.
  // With default settings (64 samples in buffer), you can spend up to 300 
  // microseconds doing something in between passing each buffer of data
  // You can tweak the buffer length to get more time if you need it.
  // When bufferlength is 64, then you get ~300 microseconds
  // When bufferlength is 128, then you get ~600 microseconds
  // Note, as you increase bufferlength, then you are increasing latency between 
  // ADC input to DAC output.
  // Latency may or may not be desired, depending on the project.
}

void codec_setup()
{
  // General setup needed
  codec.enableVREF();
  codec.enableVMID();

  // Setup signal flow to the ADC

  codec.enableLMIC();
  codec.enableRMIC();

  // Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
  codec.connectLMN1();
  codec.connectRMN1();

  // Disable mutes on PGA inputs (aka INTPUT1)
  codec.disableLINMUTE();
  codec.disableRINMUTE();

  // Set pga volumes
  codec.setLINVOLDB(0.00); // Valid options are -17.25dB to +30dB (0.75dB steps)
  codec.setRINVOLDB(0.00); // Valid options are -17.25dB to +30dB (0.75dB steps)

  // Set input boosts to get inputs 1 to the boost mixers
  codec.setLMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);
  codec.setRMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);

  // Connect from MIC inputs (aka pga output) to boost mixers
  codec.connectLMIC2B();
  codec.connectRMIC2B();

  // Enable boost mixers
  codec.enableAINL();
  codec.enableAINR();

  // Disconnect LB2LO (booster to output mixer (analog bypass)
  // For this example, we are going to pass audio throught the ADC and DAC
  codec.disableLB2LO();
  codec.disableRB2RO();

  // Connect from DAC outputs to output mixer
  codec.enableLD2LO();
  codec.enableRD2RO();

  // Set gainstage between booster mixer and output mixer
  // For this loopback example, we are going to keep these as low as they go
  codec.setLB2LOVOL(WM8960_OUTPUT_MIXER_GAIN_NEG_21DB); 
  codec.setRB2ROVOL(WM8960_OUTPUT_MIXER_GAIN_NEG_21DB);

  // Enable output mixers
  codec.enableLOMIX();
  codec.enableROMIX();

  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d 
  // freq at 705.6kHz
  codec.enablePLL(); // Needed for class-d amp clock
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h
  //codec.setADCDIV(0); // Default is 000 (what we need for 44.1KHz)
  //codec.setDACDIV(0); // Default is 000 (what we need for 44.1KHz)
  codec.setWL(WM8960_WL_16BIT);

  codec.enablePeripheralMode();
  //codec.enableMasterMode();
  //codec.setALRCGPIO(); // Note, should not be changed while ADC is enabled.

  // Enable ADCs and DACs
  codec.enableAdcLeft();
  codec.enableAdcRight();
  codec.enableDacLeft();
  codec.enableDacRight();
  codec.disableDacMute();

  //codec.enableLoopBack(); // Loopback sends ADC data directly into DAC
  codec.disableLoopBack();

  // Default is "soft mute" on, so we must disable mute to make channels active
  codec.disableDacMute(); 

  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

  Serial.println("Volume set to +0dB");
  codec.setHeadphoneVolumeDB(0.00);

  Serial.println("Codec Setup complete. Listen to left/right INPUT1 on Headphone outputs.");
}

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_driver_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
    .mclk_multiple = i2s_mclk_multiple_t(I2S_MCLK_MULTIPLE_DEFAULT),
    .bits_per_chan = i2s_bits_per_chan_t(I2S_BITS_PER_CHAN_DEFAULT)
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_SDO,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}