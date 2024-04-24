#include "node.hpp"

void Node::init_watchdog() {
#ifdef WATCHDOG_ENABLED
  // We rebooted because we got stuck or something
  if (watchdog_caused_reboot()) {
    // printf("Rebooted by Watchdog!\r\n");
    status.set(Status::Rebooted);
    sleep_ms(10000);
  }

  // Enable the watchdog, requiring the watchdog to be updated every 100ms or
  // the chip will reboot second arg is pause on debug which means the watchdog
  // will pause when stepping through code
  watchdog_enable(1000, false);
#endif
}

// void Node::watchdog_update() {
// #ifdef WATCHDOG_ENABLED
//   watchdog_update();
// #endif
// }
