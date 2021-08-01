#include <RF24.h>

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

static const uint8_t RADIO_CE_PIN = 9;
static const uint8_t RADIO_CSN_PIN = 10;
static const uint8_t CONTROLLER_TX_ADDRESS[ADDRESS_LEN + 1] = "ROBOT";
static RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

static UserInputMessage ui_msg = { 0 };

static void printUserInput(void);

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("radio hardware is not responding!!");
    while (1);
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(sizeof(ui_msg));
  radio.openReadingPipe(1, CONTROLLER_TX_ADDRESS);
  radio.startListening();
}

void loop() {
  uint8_t pipe;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize();
    radio.read(&ui_msg, bytes);
    printUserInput();
  }
}

static void printUserInput(void)
{
  Serial.print("msg tx cnt = ");
  Serial.println(ui_msg.header.tx_cnt);
  Serial.print("left (x, y, z) (");
  Serial.print(ui_msg.l_joystick.x_counts);
  Serial.print(", ");
  Serial.print(ui_msg.l_joystick.y_counts);
  Serial.print(", ");
  Serial.print(ui_msg.l_joystick.z_press_cnt);
  Serial.println(")");

  Serial.print("right (x, y, z) (");
  Serial.print(ui_msg.r_joystick.x_counts);
  Serial.print(", ");
  Serial.print(ui_msg.r_joystick.y_counts);
  Serial.print(", ");
  Serial.print(ui_msg.r_joystick.z_press_cnt);
  Serial.println(")\r\n");
}
