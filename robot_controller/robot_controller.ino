#include <RF24.h>
#include <ArducamSSD1306.h>

#define ADDRESS_LEN 5

typedef struct __attribute__((packed))
{
  uint8_t id;
  uint8_t version;
  uint8_t tx_cnt;
} MessageHeader;

typedef struct __attribute__((packed))
{
  uint16_t x_counts;
  uint16_t y_counts;
  uint8_t z_press_cnt;
} JoystickInput;

typedef struct __attribute__((packed))
{
  MessageHeader header;
  JoystickInput l_joystick;
  JoystickInput r_joystick;
} UserInputMessage;

static const uint8_t R_X_PIN = A1;
static const uint8_t R_Y_PIN = A0;
static const uint8_t R_Z_PIN = 2;

static const uint8_t L_X_PIN = A2;
static const uint8_t L_Y_PIN = A3;
static const uint8_t L_Z_PIN = 3;

static const int8_t OLED_RESET = -1;  // No hardware reset support
static ArducamSSD1306 display(OLED_RESET);

static const uint8_t RADIO_CE_PIN = 9;
static const uint8_t RADIO_CSN_PIN = 10;
static const uint8_t CONTROLLER_TX_ADDRESS[ADDRESS_LEN + 1] = "ROBOT";
static RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

static UserInputMessage ui_msg = { 0 };

static volatile uint32_t sys_tick = 0;

void initSysTicker(void);
uint32_t getSysTick(void);

void setup() {
  Serial.begin(115200);

  // Initialize the RF24 radio
  const bool radio_init_success = radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(sizeof(UserInputMessage));
  radio.openWritingPipe(CONTROLLER_TX_ADDRESS);
  radio.stopListening();

  // Initialize the OLED display
  display.begin();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,20);

  // Initialize the system ticker
  initSysTicker();

  // Attach interrupts to the z button on the joysticks
  pinMode(R_Z_PIN, INPUT_PULLUP);
  pinMode(L_Z_PIN, INPUT_PULLUP);

  if (radio_init_success) {
    display.println("init success");
    display.display();
  } else {
    display.println("rf24 init fail");
    display.display();
    while(1);
  }

  ui_msg.header.id = 0xA5;
  ui_msg.header.version = 0x01u;
}

void loop() {
  static uint32_t task_100ms_trigger_ticks = 0;
  static uint8_t r_closed_cnt = 0;
  static uint8_t l_closed_cnt = 0;

  const uint32_t cur_sys_tick = getSysTick();

  if (LOW == digitalRead(R_Z_PIN)) {
    // switch closed
    r_closed_cnt++;
  }

  if (LOW == digitalRead(L_Z_PIN)) {
    // switch closed
    l_closed_cnt++;
  }

  if (cur_sys_tick > task_100ms_trigger_ticks) {
    task_100ms_trigger_ticks += 100;

    if (r_closed_cnt > 100)
    {
      ui_msg.r_joystick.z_press_cnt++;
    }

    if (l_closed_cnt > 100)
    {
      ui_msg.l_joystick.z_press_cnt++;
    }

    r_closed_cnt = 0;
    l_closed_cnt = 0;

    ui_msg.l_joystick.x_counts = analogRead(L_X_PIN);
    ui_msg.l_joystick.y_counts = analogRead(L_Y_PIN);

    ui_msg.r_joystick.x_counts = analogRead(R_X_PIN);
    ui_msg.r_joystick.y_counts = analogRead(R_Y_PIN);

    unsigned long start_timer = micros();
    bool report = radio.write(&ui_msg, sizeof(UserInputMessage));
    unsigned long end_timer = micros();
    ui_msg.header.tx_cnt++;

    if (report) {
      Serial.print("Transmission successful! ");
      Serial.print("Time to transmit = ");
      Serial.print(end_timer - start_timer);
      Serial.println(" us.");
    } else {
      Serial.println("Transmission failed or timed out");
    }
  }
}

void initSysTicker(void) {
  TCCR1A = 0;  // set entire TCCR0A register to 0
  TCCR1B = 0;  // same for TCCR0B
  TCNT1  = 0;  // initialize counter value to 0

  // Set compare match register for 1khz increments
  OCR1A = 249;  // = (16e6) / (1000 * 64) - 1 (must be < 256)

  // Turn on CTC mode
  TCCR1A |= (1 << WGM01);

  // Set CS01 and CS00 bits for 64 prescaler
  TCCR1B |= (1 << CS01) | (1 << CS00);

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE0A);
}

ISR(TIMER1_COMPA_vect) {
  sys_tick++;
}

uint32_t getSysTick(void) {
  cli();  // disable interrupts
  const uint32_t temp = sys_tick;
  sei();  // enable interrupts
  return temp;
}
