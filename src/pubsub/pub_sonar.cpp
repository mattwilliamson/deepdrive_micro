#include "pubsub/pub_sonar.hpp"

// Map of gpio number to triggered sonar sensors
static bool _pub_sonar_triggered[30];

bool PubSonar::trigger(repeating_timer_t *rt) {
  _pub_sonar_triggered[(uint)rt->user_data] = true;
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

  msg_ = sensor_msgs__msg__LaserScan__create();
  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);
  // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html
  msg_->angle_min = -SONAR_FOV * M_PI / 180;                          // degrees in radians start of scan
  msg_->angle_max = SONAR_FOV * M_PI / 180;                           // degrees in radians end of scan
  msg_->angle_increment = SONAR_FOV * M_PI / 180 / SONAR_LASER_RAYS;  // degrees in radians
  msg_->time_increment = 0.0;                                         // time between measurements [seconds]
  msg_->scan_time = 1.0 / timer_hz;                                   // time between scans [seconds]
  msg_->range_min = SONAR_MIN_DISTANCE;                               // minimum range value [m]
  msg_->range_max = SONAR_MAX_DISTANCE;                               // maximum range value [m]
  assert(rosidl_runtime_c__float32__Sequence__init(&msg_->ranges, SONAR_LASER_RAYS));
  assert(rosidl_runtime_c__float32__Sequence__init(&msg_->intensities, 0));

  pin_trigger_ = trigger_pin;
  pin_echo_ = echo_pin;
  sample_index_ = 0;

  for (int i = 0; i < SONAR_SAMPLES; i++) {
    samples_[i] = 0;
  }

  sonar_ = new HCSR04(echo_pin, trigger_pin);

  mutex_init(&lock_);

  data_ready_ = false;
  _pub_sonar_triggered[pin_trigger_] = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      topic_name);
  assert(status_ == RCL_RET_OK);

  // Clear FIFO
  // sonar_->trigger();

  assert(add_repeating_timer_us(-MICROSECONDS / timer_hz, PubSonar::trigger, (void *)trigger_pin, &timer_));
  started_ = true;
}

void PubSonar::calculate() {
  mutex_enter_blocking(&lock_);

  if (started_ != true) {
    mutex_exit(&lock_);
    return;
  }

  if (!_pub_sonar_triggered[pin_trigger_]) {
    mutex_exit(&lock_);
    return;
  }

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

  // Default to nothing detected
  double avg = infinity();

  // Only use the samples that didn't timeout
  // TODO: Maybe we want to include these and only filter the average?
  if (good_samples > 0) {
    // Get the average of all the samples
    avg = sum / good_samples;
  }

  for (int i = 0; i < SONAR_LASER_RAYS; i++) {
    msg_->ranges.data[i] = avg;
  }

  // Message is ready to publish
  PubSub::set_timestamp_header(&msg_->header);
  data_ready_ = true;

  // Reset trigger for next loop
  _pub_sonar_triggered[pin_trigger_] = false;
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