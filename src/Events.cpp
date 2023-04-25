#include "Events.h"

Event::Event(unsigned long interval, bool repeat, void (*callback)(void)) {
  this->interval = interval;
  this->repeat = repeat;
  this->callback = callback;
  this->lastRun = millis();
}

bool Event::check() {
  unsigned long now = millis();
  if (now - lastRun >= interval) {
    if (!repeat) {
      interval = 0; // Mark as expired
    }
    lastRun = now;
    return true;
  }
  return false;
}

EventScheduler::EventScheduler() : eventCount(0) {}

bool EventScheduler::addEvent(unsigned long interval, bool repeat, void (*callback)(void)) {
  if (eventCount == maxEvents) {
    return false;
  }
  events[eventCount++] = new Event(interval, repeat, callback);
  return true;
}

void EventScheduler::run() {
  for (byte i = 0; i < eventCount; i++) {
    if (events[i]->check()) {
      events[i]->callback();
      if (events[i]->interval == 0) {
        delete events[i];
        for (byte j = i + 1; j < eventCount; j++) {
          events[j - 1] = events[j];
        }
        eventCount--;
        i--;
      }
    }
  }
}
