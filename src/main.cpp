#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

// WS2812 RGB LED support (WAVESHARE_RP2040 only)
#if defined(ARDUINO_WAVESHARE_RP2040_ZERO)
  #include "ws2812.pio.h"
  #define WS2812_PIN 16  // RGB LED pin on WAVESHARE_RP2040
  #define LED_ENABLED 1
#elif defined(ARDUINO_SEEED_XIAO_RP2040)
  #include "ws2812.pio.h"
  #define WS2812_PIN 12  // RGB LED pin on XIAO_RP2040
  #define LED_ENABLED 1
#else
  #define LED_ENABLED 0
#endif

// USB HID report descriptor
// Specifies the structure of the gamepad data (for radio receiver 
// (16bit data x 8ch) + (1bit data x 8ch))
#define TUD_HID_REPORT_DESC_GAMEPAD_9(...)                                    \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                                     \
      HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD),                                   \
      HID_COLLECTION(HID_COLLECTION_APPLICATION), /* Report ID if any */      \
      __VA_ARGS__ HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                     \
      HID_USAGE(HID_USAGE_DESKTOP_X), HID_USAGE(HID_USAGE_DESKTOP_Y),         \
      HID_USAGE(HID_USAGE_DESKTOP_Z), HID_USAGE(HID_USAGE_DESKTOP_RX),        \
      HID_USAGE(HID_USAGE_DESKTOP_RY), HID_USAGE(HID_USAGE_DESKTOP_RZ),       \
      HID_USAGE(HID_USAGE_DESKTOP_SLIDER), HID_USAGE(HID_USAGE_DESKTOP_DIAL), \
      HID_LOGICAL_MIN(0), /* HID_LOGICAL_MAX ( 0x7ff ) ,*/                    \
      HID_LOGICAL_MAX_N(0xffff, 3), HID_REPORT_COUNT(8), HID_REPORT_SIZE(16), \
      HID_INPUT(HID_DATA | HID_VARIABLE |                                     \
                HID_ABSOLUTE), /* 8 bit Button Map */                         \
      HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON), HID_USAGE_MIN(1),                \
      HID_USAGE_MAX(8), HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(1),               \
      HID_REPORT_COUNT(8), HID_REPORT_SIZE(1),                                \
      HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                      \
      HID_COLLECTION_END  

// CrossFire settings
#define CRSF_BAUDRATE 420000
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_NUM_CHANNELS 16

// LED Status States
typedef enum {
  LED_STATUS_BOOT = 0,           // Yellow - Booting/initializing
  LED_STATUS_USB_WAIT,           // Yellow pulse - USB mounted, waiting for CRSF
  LED_STATUS_ACTIVE,             // Green - Active link with valid data
  LED_STATUS_STALE,              // Orange - No recent frames
  LED_STATUS_NO_SIGNAL,          // Red - No CRSF signal
  LED_STATUS_USB_SUSPENDED,      // Red dim - USB suspended
  LED_STATUS_FIRMWARE_UPDATE,    // Blue pulse - Firmware update mode
  LED_STATUS_ERROR               // Red fast blink - Error/invalid frames
} led_status_e;

// LED Color definitions (GRB format for WS2812)
#define LED_COLOR_OFF       0x000000
#define LED_COLOR_RED       0x001000
#define LED_COLOR_GREEN     0x100000
#define LED_COLOR_BLUE      0x000010
#define LED_COLOR_YELLOW    0x100800
#define LED_COLOR_ORANGE    0x080800
#define LED_COLOR_CYAN      0x100010
#define LED_COLOR_PURPLE    0x051005
#define LED_COLOR_WHITE     0x101010
#define LED_COLOR_DIM_RED   0x000400
#define LED_COLOR_DIM_GREEN 0x040000

typedef enum {
  CRSF_ADDRESS_BROADCAST = 0x00,
  CRSF_ADDRESS_USB = 0x10,
  CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
  CRSF_ADDRESS_RESERVED1 = 0x8A,
  CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
  CRSF_ADDRESS_GPS = 0xC2,
  CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
  CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,  // Incoming data comes in this
  CRSF_ADDRESS_RESERVED2 = 0xCA,
  CRSF_ADDRESS_RACE_TAG = 0xCC,
  CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
  CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
  CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;

typedef enum {
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
  CRSF_FRAMETYPE_RADIO_ID = 0x3A,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,  // packed channel frame
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  // Extended Header Frames, range: 0x28 to 0x96
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
  CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
  CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
  CRSF_FRAMETYPE_COMMAND = 0x32,
  // MSP commands
  CRSF_FRAMETYPE_MSP_REQ =  0x7A,   // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B,   // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_e;

typedef struct crsf_header_s {
  uint8_t device_addr;  // from crsf_addr_e
  uint8_t frame_size;   // counts size after this byte, so it must be the payload size + 2 (type and crc)
  uint8_t type;         // from crsf_frame_type_e
  uint8_t data[0];
} crsf_header_t;

Adafruit_USBD_HID usb_hid;  // USB HID object
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD_9()  // USB GamePad data structure
};

// Define CRSF serial port based on board type
#ifdef ARDUINO_RASPBERRY_PI_PICO
  #define CRSF_SERIAL Serial2
#else
  #define CRSF_SERIAL Serial1
#endif

typedef struct __attribute__((packed)) gamepad_data {
  uint16_t ch[8];  // 16 bit 8ch
  uint8_t sw;      //  1 bit 8ch
} gp_t;

uint8_t rxbuf[CRSF_MAX_PACKET_LEN + 3];  // Raw data received from the CRSF Rx
uint8_t rxPos = 0;
static gamepad_data gp;  // Data sorted by channel
uint8_t frameSize = 0;
int datardyf = 0;        // Data read to send to USB
uint32_t gaptime;        // For bus delimiter measurement

// LED status tracking
#if LED_ENABLED
PIO pio = pio0;
uint sm = 0;
led_status_e led_status = LED_STATUS_BOOT;
uint32_t last_valid_frame_time = 0;
uint32_t led_update_time = 0;
bool in_firmware_update = false;
#endif

#if defined(DEBUG)
uint32_t time_m;         // Interval time (for debug)
void debug_out();
#endif

void crsf();
void crsfdecode();
void uart();

#if LED_ENABLED
void led_init();
void led_set_color(uint32_t color);
void led_update();
#endif

void setup()
{
#if LED_ENABLED
  // Initialize LED first to show boot status
  led_init();
  led_set_color(LED_COLOR_YELLOW);  // Boot status
#endif

  // USB HID Device Settings
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();
  while (!USBDevice.mounted()) delay(1);  // wait until device mounted

  datardyf = 0;
  gaptime = 0;
  rxPos = 0;

  // For PC serial communication (to match receiver speed)
  Serial.begin(CRSF_BAUDRATE);
  // For CRSF communication (420kbps, 8bit data, nonParity, 1 stopbit)
#ifdef ARDUINO_RASPBERRY_PI_PICO
  // For PICO board: set TX(4) and RX(5) pins
  CRSF_SERIAL.setTX(4);
  CRSF_SERIAL.setRX(5);
#endif
  CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1);

#if LED_ENABLED
  // USB mounted, waiting for CRSF data
  led_status = LED_STATUS_USB_WAIT;
  last_valid_frame_time = millis();
#endif

#if defined(DEBUG)
  time_m = micros();  // For interval measurement
#endif
}

void loop()
{
  // WAke up host if in suspend mode and REMOTE_WAKEUP feature by host
  if (USBDevice.suspended()) {
    USBDevice.remoteWakeup();
#if LED_ENABLED
    if (led_status != LED_STATUS_USB_SUSPENDED && led_status != LED_STATUS_FIRMWARE_UPDATE) {
      led_status = LED_STATUS_USB_SUSPENDED;
    }
#endif
  }
  crsf();           // Process CRSF
  uart();           // UART communication processing (for firmware rewriting)
  if (datardyf) {   // USB transmission when data is ready
    if (usb_hid.ready()) {
      usb_hid.sendReport(0, &gp, sizeof(gp));
#ifdef DEBUG
      debug_out();  // For debugging (check values on serial monitor)
#endif
    }
    datardyf = 0;  // Clear the flag after transmission
  }

#if LED_ENABLED
  // Update LED status based on link quality
  if (!in_firmware_update) {
    uint32_t current_time = millis();
    uint32_t time_since_frame = current_time - last_valid_frame_time;

    if (USBDevice.suspended()) {
      if (led_status != LED_STATUS_USB_SUSPENDED) {
        led_status = LED_STATUS_USB_SUSPENDED;
      }
    } else if (time_since_frame < 100) {
      // Active link - received frame in last 100ms
      if (led_status != LED_STATUS_ACTIVE) {
        led_status = LED_STATUS_ACTIVE;
      }
    } else if (time_since_frame < 1000) {
      // Stale link - no frame for 100-1000ms
      if (led_status != LED_STATUS_STALE) {
        led_status = LED_STATUS_STALE;
      }
    } else {
      // No signal - no frame for >1 second
      if (led_status != LED_STATUS_NO_SIGNAL) {
        led_status = LED_STATUS_NO_SIGNAL;
      }
    }
  }

  led_update();  // Update LED based on current status
#endif
}

//CRSF receive process
void crsf()
{
  uint8_t data;
  // byte received from CRSF
  if (CRSF_SERIAL.available()) {  // If there is incoming data on CRSF serial
    data = CRSF_SERIAL.read();    // 8-bit data read
    gaptime = micros();
    if (rxPos == 1) {
      frameSize = data;       // Second byte is the frame size
    }
    rxbuf[rxPos++] = data;    // Store received data in buffer
    if (rxPos > 1 && rxPos >= frameSize + 2) {
      crsfdecode();           // Decode after receiving one frame
      rxPos = 0;
    }
  } else {
    // If no data comes in for more than 800us, judge as a break
    if (rxPos > 0 && micros() - gaptime > 800) {
      rxPos = 0;
    }
  }
}

// Decode 11-bit serial data received from CRSF to 16-bit data (further bottom up. 5-bit)
// sift)
void crsfdecode()
{
  if (rxbuf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER) {       // header check
    if (rxbuf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {  // Decode if CH data
      gp.sw = 0;
      gp.ch[0] = ((rxbuf[3] | rxbuf[4] << 8) & 0x07ff) << 5;
      gp.ch[1] = ((rxbuf[4] >> 3 | rxbuf[5] << 5) & 0x07ff) << 5;
      gp.ch[2] = ((rxbuf[5] >> 6 | rxbuf[6] << 2 | rxbuf[7] << 10) & 0x07ff)
                 << 5;
      gp.ch[3] = ((rxbuf[7] >> 1 | rxbuf[8] << 7) & 0x07ff) << 5;
      if (((rxbuf[8] >> 4 | rxbuf[9] << 4) & 0x07ff) > 0x3ff)
        gp.sw |= 0x01;  //  AUX1 is binary data
      gp.ch[4] = ((rxbuf[9] >> 7 | rxbuf[10] << 1 | rxbuf[11] << 9) & 0x07ff)
                 << 5;
      gp.ch[5] = ((rxbuf[11] >> 2 | rxbuf[12] << 6) & 0x07ff) << 5;
      gp.ch[6] = ((rxbuf[12] >> 5 | rxbuf[13] << 3) & 0x07ff) << 5;
      gp.ch[7] = ((rxbuf[14] | rxbuf[15] << 8) & 0x7ff) << 5;
      if (((rxbuf[15] >> 3 | rxbuf[16] << 5) & 0x07ff) > 0x3ff) gp.sw |= 0x02;
      if (((rxbuf[16] >> 6 | rxbuf[17] << 2 | rxbuf[18] << 10) & 0x07ff) >
          0x3ff)
        gp.sw |= 0x04;
      if (((rxbuf[18] >> 1 | rxbuf[19] << 7) & 0x07ff) > 0x3ff) gp.sw |= 0x08;
      if (((rxbuf[19] >> 4 | rxbuf[20] << 4) & 0x07ff) > 0x3ff) gp.sw |= 0x10;
      if (((rxbuf[20] >> 7 | rxbuf[21] << 1 | rxbuf[22] << 9) & 0x07ff) > 0x3ff)
        gp.sw |= 0x20;
      if (((rxbuf[22] >> 2 | rxbuf[23] << 6) & 0x07ff) > 0x3ff) gp.sw |= 0x40;
      if (((rxbuf[23] >> 5 | rxbuf[24] << 3) & 0x07ff) > 0x3ff) gp.sw |= 0x80;

      datardyf = 1;  // set data ready flag

#if LED_ENABLED
      // Update timestamp on successful frame decode
      last_valid_frame_time = millis();
#endif
    }
  }
}

// UART communication process (for firmware rewriting)
// Flashing Method in ExpressLRS Configurator should be [BetaflightPassthough], not [UART].
void uart()
{
  uint32_t t;

  // When data comes from the PC, it is determined to be in firmware update mode
  if (Serial.available()) {
#if LED_ENABLED
    in_firmware_update = true;
    led_status = LED_STATUS_FIRMWARE_UPDATE;
#endif

    t = millis();
    do {
      while (Serial.available()) {     // When data comes in from the PC
        CRSF_SERIAL.write(Serial.read());  // Send data from PC to receiver
        t = millis();
      }
      while (CRSF_SERIAL.available()) {    // When data comes in from the receiver
        Serial.write(CRSF_SERIAL.read());  // Send receiver data to PC
        t = millis();
      }
    } while (millis() - t < 2000);     // When data stops coming in, it's over

#if LED_ENABLED
    in_firmware_update = false;
    led_status = LED_STATUS_USB_WAIT;
    last_valid_frame_time = millis();  // Reset timer after firmware update
#endif
  }
}

#if defined(DEBUG)
// Display received data on serial monitor (for debugging)
void debug_out()
{
  int i;
  Serial.print(rxbuf[0], HEX);  // device addr
  Serial.print(" ");
  Serial.print(rxbuf[1]);       // data size +1
  Serial.print(" ");
  Serial.print(rxbuf[2], HEX);  // type
  Serial.print(" ");
  for (i = 0; i < 8; i++) {
    Serial.print(gp.ch[i]);
    Serial.print(" ");
  }
  Serial.print(gp.sw, BIN);
  Serial.print(" ");
  Serial.print(micros() - time_m);  // Display interval time (us)
  Serial.println("us");
  time_m = micros();
}
#endif

#if LED_ENABLED
// Initialize WS2812 LED using PIO
void led_init()
{
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);
}

// Set LED to specific color (GRB format)
void led_set_color(uint32_t color)
{
  pio_sm_put_blocking(pio, sm, color << 8u);
}

// Update LED based on current status with appropriate patterns
void led_update()
{
  static uint32_t last_update = 0;
  static bool blink_state = false;
  uint32_t current_time = millis();

  // Update at different rates based on status
  uint32_t update_interval;
  switch (led_status) {
    case LED_STATUS_BOOT:
      update_interval = 500;  // Slow blink during boot
      break;
    case LED_STATUS_USB_WAIT:
      update_interval = 1000;  // Slow pulse waiting for CRSF
      break;
    case LED_STATUS_ACTIVE:
      // Solid green, no blinking needed
      if (current_time - led_update_time > 100) {
        led_set_color(LED_COLOR_GREEN);
        led_update_time = current_time;
      }
      return;
    case LED_STATUS_STALE:
      update_interval = 500;  // Medium blink for stale link
      break;
    case LED_STATUS_NO_SIGNAL:
      update_interval = 1000;  // Slow blink for no signal
      break;
    case LED_STATUS_USB_SUSPENDED:
      // Dim red, solid
      if (current_time - led_update_time > 100) {
        led_set_color(LED_COLOR_DIM_RED);
        led_update_time = current_time;
      }
      return;
    case LED_STATUS_FIRMWARE_UPDATE:
      update_interval = 200;  // Fast pulse during firmware update
      break;
    case LED_STATUS_ERROR:
      update_interval = 100;  // Very fast blink for errors
      break;
    default:
      update_interval = 500;
      break;
  }

  if (current_time - last_update >= update_interval) {
    last_update = current_time;
    blink_state = !blink_state;

    uint32_t color;
    switch (led_status) {
      case LED_STATUS_BOOT:
      case LED_STATUS_USB_WAIT:
        color = blink_state ? LED_COLOR_YELLOW : LED_COLOR_OFF;
        break;
      case LED_STATUS_STALE:
        color = blink_state ? LED_COLOR_ORANGE : LED_COLOR_OFF;
        break;
      case LED_STATUS_NO_SIGNAL:
        color = blink_state ? LED_COLOR_RED : LED_COLOR_OFF;
        break;
      case LED_STATUS_FIRMWARE_UPDATE:
        color = blink_state ? LED_COLOR_BLUE : LED_COLOR_CYAN;
        break;
      case LED_STATUS_ERROR:
        color = blink_state ? LED_COLOR_RED : LED_COLOR_OFF;
        break;
      default:
        color = LED_COLOR_OFF;
        break;
    }

    led_set_color(color);
  }
}
#endif
