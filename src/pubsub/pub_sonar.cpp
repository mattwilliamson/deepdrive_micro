#include "pubsub/pub_sonar.hpp"

static bool _pub_sonar_triggered;

bool PubSonar::trigger(repeating_timer_t *rt) {
  _pub_sonar_triggered = true;
  return true;
}

PubSonar::PubSonar(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                   uint trigger_pin,
                   uint echo_pin,
                   int64_t timer_hz,
                   const char *topic_name,
                   const char *frame_id) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;

  // sensor_msgs__msg__LaserScan__create()
  msg_ = sensor_msgs__msg__Range__create();
  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);
  // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html
  msg_->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  msg_->field_of_view = SONAR_FOV * M_PI / 180;  // degrees in radians
  msg_->min_range = SONAR_MIN_DISTANCE;
  msg_->max_range = SONAR_MAX_DISTANCE;
  msg_->range = 0.0;

  pin_trigger_ = trigger_pin;
  pin_echo_ = echo_pin;
  sample_index_ = 0;

  for (int i = 0; i < SONAR_SAMPLES; i++) {
    samples_[i] = 0;
  }

  sonar_ = new HCSR04(pin_echo_, pin_trigger_);

  mutex_init(&lock_);

  data_ready_ = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      topic_name);
  assert(status_ == RCL_RET_OK);

  // Clear FIFO
  sonar_->trigger();

  assert(add_repeating_timer_us(-MICROSECONDS / timer_hz, PubSonar::trigger, NULL, &timer_));
}

void PubSonar::calculate() {
  if (!_pub_sonar_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

  // Wrap around the ring buffer
  if (sample_index_ >= SONAR_SAMPLES) {
    sample_index_ = 0;
  }

  // Read the sonar distance and stick it in the buffer
  double s = sonar_->read();
  samples_[sample_index_] = s;

  //  Average out the readings
  uint good_samples = 0;
  double sum = 0;

  for (int i = 0; i < SONAR_SAMPLES; i++) {
    double s = samples_[i];
    if (s > SONAR_MIN_DISTANCE && s < SONAR_MAX_DISTANCE) {
      good_samples++;
      sum += s;
    }
  }

  // Only use the samples that didn't timeout
  // TODO: Maybe we want to include these and only filter the average?
  if (good_samples > 0) {
    // Get the average of all the samples
    msg_->range = sum / good_samples;
  } else {
    // No good ones
    msg_->range = infinity();
  }

  // Message is ready to publish
  PubSub::set_timestamp_header(&msg_->header);
  data_ready_ = true;

  // Reset trigger for next loop
  _pub_sonar_triggered = false;
  sample_index_++;

  // Reset FIFO queue to get new readings
  // sonar_->trigger();

  // TODO: switch between front and back sensors by swapping out pins assigned to state machines
  //   // set the 'in' pins, also used for 'wait'
  // sm_config_set_in_pins(&c, input);
  // // set the 'jmp' pin
  // sm_config_set_jmp_pin(&c, input);

  mutex_exit(&lock_);
}

void PubSonar::publish() {
  mutex_enter_blocking(&lock_);
  if (data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
  }
  mutex_exit(&lock_);
}

PubSonar::~PubSonar() {
  cancel_repeating_timer(&timer_);
  status_ = rcl_publisher_fini(&publisher_, node_);
}