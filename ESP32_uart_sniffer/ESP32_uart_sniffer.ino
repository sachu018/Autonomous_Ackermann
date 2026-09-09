/* ============================================================================
 *  ESP32_uart_sniffer.ino — passive listener for the STM32 -> RPi feedback
 *  link, used to verify the physical/electrical UART connection independent
 *  of the RPi software stack.
 * ============================================================================
 *
 *  WIRING — READ-ONLY TAP, ONE WAY ONLY:
 *      STM32 PA2 (USART2_TX)  ->  ESP32 GPIO16 (RX2)
 *      STM32 GND              ->  ESP32 GND
 *
 *  Do NOT connect anything to STM32 PA3 (USART2_RX) while the RPi is also
 *  wired to it. PA3 is a single wire — if both the RPi's TX pin and this
 *  ESP32's TX pin drive it at once, both boards' UART transmitters fight
 *  each other electrically. This sketch's own TX pin (GPIO17) is configured
 *  in software but deliberately left UNCONNECTED — this is a listen-only
 *  tap, exactly the same one-way-bridge safety rule already used for the
 *  PA9 debug telemetry link elsewhere in this project (see Rover_study.md
 *  §2.7: "the bridge is deliberately one-way").
 *
 *  WHAT THIS PROVES: whether the STM32 is actually putting feedback frames
 *  out on PA2 at all — i.e. the electrical/physical link, completely
 *  independent of the RPi's Python code, its serial port config, or
 *  anything else on that side. If frames show up here but check_stm32_link.py
 *  on the RPi still sees nothing, the problem is specifically on the RPi
 *  side (wiring into the Pi's header, or the Pi's own UART config) rather
 *  than the STM32. If NOTHING shows up here either, the problem is on the
 *  STM32 side or the STM32-to-tap-point wiring itself.
 *
 *  Frame format — MUST stay identical to Rover_closed_loop/Core/Src/rpi_link.c's
 *  header comment and RPi_companion/uart_link.py. No shared schema file
 *  between the three; keep all in sync by hand if the protocol ever changes.
 *
 *  Feedback frame, STM32 -> RPi (and now -> this sniffer), 12 bytes, LE:
 *      [0]     0xBB                    header
 *      [1]     0x66                    header
 *      [2..3]  int16  angle_L          degrees x100
 *      [4..5]  int16  angle_R          degrees x100
 *      [6..7]  int16  rpm_L            RPM x10
 *      [8..9]  int16  rpm_R            RPM x10
 *      [10]    uint8  status           bitfield, see STATUS_* below
 *      [11]    uint8  checksum         XOR of bytes [0..10]
 *
 *  Board: any ESP32 Dev Module. Arduino IDE, no extra libraries needed.
 *  Open the Serial Monitor at 115200 baud after uploading.
 * ============================================================================
 */

#include <HardwareSerial.h>

// UART2 on the ESP32 — GPIO16 = RX2, GPIO17 = TX2 (TX left unconnected, see
// the wiring note above; WROVER boards use GPIO16/17 for PSRAM, use e.g.
// GPIO25/26 instead there — see Rover_study.md §10.4's identical caveat for
// the existing PA9 debug bridge).
#define STM32_RX_PIN 16
#define STM32_TX_PIN 17
#define STM32_BAUD   115200

HardwareSerial STM32Serial(2);

static const uint8_t FEEDBACK_FRAME_LEN = 12;
static const uint8_t HDR0 = 0xBB;
static const uint8_t HDR1 = 0x66;

#define STATUS_ARMED       (1 << 0)
#define STATUS_STEER_FAULT (1 << 1)
#define STATUS_AUTO_ACTIVE (1 << 2)
#define STATUS_RC_OK       (1 << 3)

uint8_t buf[64];
uint8_t buf_len = 0;

uint32_t frames_ok = 0;
uint32_t frames_bad = 0;
uint32_t t_start = 0;
uint32_t t_last_print = 0;
uint32_t t_last_frame = 0;

uint8_t checksum(const uint8_t *b, uint8_t len) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < len; i++) x ^= b[i];
  return x;
}

void handleFrame(const uint8_t *f) {
  int16_t angle_L = (int16_t)(f[2] | (f[3] << 8));
  int16_t angle_R = (int16_t)(f[4] | (f[5] << 8));
  int16_t rpm_L   = (int16_t)(f[6] | (f[7] << 8));
  int16_t rpm_R   = (int16_t)(f[8] | (f[9] << 8));
  uint8_t status  = f[10];

  frames_ok++;
  t_last_frame = millis();

  Serial.printf(
    "OK  angleL=%+6.2f angleR=%+6.2f | rpmL=%+6.1f rpmR=%+6.1f | "
    "ARMED=%d STEER_FAULT=%d AUTO_ACTIVE=%d RC_OK=%d\n",
    angle_L / 100.0, angle_R / 100.0, rpm_L / 10.0, rpm_R / 10.0,
    (status & STATUS_ARMED) ? 1 : 0,
    (status & STATUS_STEER_FAULT) ? 1 : 0,
    (status & STATUS_AUTO_ACTIVE) ? 1 : 0,
    (status & STATUS_RC_OK) ? 1 : 0
  );
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[Sniffer] ESP32 UART sniffer for STM32 PA2 (USART2_TX) feedback frames.");
  Serial.println("[Sniffer] Wiring: STM32 PA2 -> ESP32 GPIO16, STM32 GND -> ESP32 GND.");
  Serial.println("[Sniffer] PA3 must stay disconnected from this board — see header comment.");
  Serial.println();

  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  t_start = millis();
  t_last_print = t_start;
}

void loop() {
  // Pull in whatever bytes are available, append to our small buffer.
  while (STM32Serial.available() && buf_len < sizeof(buf)) {
    buf[buf_len++] = (uint8_t)STM32Serial.read();
  }

  // Scan for a complete, valid frame anywhere in the buffer, resyncing
  // byte-by-byte on a checksum mismatch (no IDLE-line hardware framing
  // available on this side, same approach as uart_link.py on the RPi).
  bool progressed = true;
  while (progressed) {
    progressed = false;

    // Find header
    uint8_t start = 0xFF;
    for (uint8_t i = 0; i + 1 < buf_len; i++) {
      if (buf[i] == HDR0 && buf[i + 1] == HDR1) { start = i; break; }
    }

    if (start == 0xFF) {
      // No header found — keep only the last byte in case it's a split header.
      if (buf_len > 1) { buf[0] = buf[buf_len - 1]; buf_len = 1; }
      break;
    }

    if (start > 0) {
      // Drop garbage before the header.
      memmove(buf, buf + start, buf_len - start);
      buf_len -= start;
      progressed = true;
      continue;
    }

    if (buf_len < FEEDBACK_FRAME_LEN) break;  // wait for more bytes

    if (checksum(buf, FEEDBACK_FRAME_LEN - 1) == buf[FEEDBACK_FRAME_LEN - 1]) {
      handleFrame(buf);
      memmove(buf, buf + FEEDBACK_FRAME_LEN, buf_len - FEEDBACK_FRAME_LEN);
      buf_len -= FEEDBACK_FRAME_LEN;
    } else {
      frames_bad++;
      memmove(buf, buf + 2, buf_len - 2);  // drop just the header, keep scanning
      buf_len -= 2;
    }
    progressed = true;
  }

  // Periodic status line, especially useful while nothing is arriving.
  uint32_t now = millis();
  if (now - t_last_print >= 1000) {
    t_last_print = now;
    float elapsed_s = (now - t_start) / 1000.0;
    float rate = frames_ok / elapsed_s;
    uint32_t since_last = t_last_frame ? (now - t_last_frame) : 0;
    Serial.printf(
      "[Sniffer] t=%.1fs  frames_ok=%lu  frames_bad=%lu  rate=%.1f/s  "
      "last_frame=%lums ago  %s\n",
      elapsed_s, frames_ok, frames_bad, rate, since_last,
      frames_ok == 0 ? "-- NOTHING RECEIVED YET --" : ""
    );
  }
}
