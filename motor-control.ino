#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <stdarg.h>

class Msg
{
public:
  enum class Motor : uint8_t
  {
    Steering = 0,
    Throttle = 1,
  };

  using Serialized = std_msgs::UInt8;

  Motor motor;
  uint8_t position;

  Msg(Serialized const& serialized)
  : motor((Motor)(serialized.data >> 7)), position(serialized.data & 0x7f)
  {}
};

ros::NodeHandle node;

void mcu_cb(Msg::Serialized const& mcu_msg);
ros::Subscriber<Msg::Serialized> mcu("mcu", &mcu_cb);

std_msgs::String mcu_dbg_msg;
ros::Publisher mcu_dbg("mcu_dbg", &mcu_dbg_msg);

static size_t constexpr steering_pin = 4;
Servo steering;
static size_t constexpr throttle_pin = 6;
Servo throttle;

void blink(size_t n, size_t delay_ms)
{
  for (; n > 0; n--)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_ms);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_ms);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void debug(char const* fmt, ...)
{
  static size_t constexpr buf_len = 128;
  static char buf[buf_len];
  mcu_dbg_msg.data = buf;
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, buf_len, fmt, args);
  va_end(args);
  mcu_dbg.publish(&mcu_dbg_msg);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  steering.attach(steering_pin);
  throttle.attach(throttle_pin);

  node.initNode();
  node.advertise(mcu_dbg);
  node.subscribe(mcu);

  blink(3, 200);
}

void loop()
{
  node.spinOnce();
}

static void set_motor_position(Msg msg)
{
  static_assert(sizeof(size_t) > 1);
  static size_t constexpr offset_deg = 30;
  static size_t constexpr neutral_deg = 60;

  if (msg.position > 120)
  {
    debug("position out of range: %d", (size_t)msg.position);
    msg.position = neutral_deg;
  }

  size_t position_deg = msg.position + offset_deg;
  switch (msg.motor)
  {
    case Msg::Motor::Steering:
      steering.write(position_deg);
      break;
    case Msg::Motor::Throttle:
      throttle.write(position_deg);
      break;
    default:
      debug("unexpected motor: %d", (size_t)msg.motor);
  }
}

void mcu_cb(Msg::Serialized const& mcu_msg)
{
  const Msg msg(mcu_msg);
  set_motor_position(msg);

  debug(
    "msg: %d, motor %d, position %d",
    (size_t)mcu_msg.data,
    (size_t)msg.motor,
    (size_t)msg.position);
}
