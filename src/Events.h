#ifndef EVENT_SCHEDULER_H
#define EVENT_SCHEDULER_H
#include <Arduino.h>

class Event {
  public:
    unsigned long interval;
    unsigned long lastRun;
    bool repeat;
    void (*callback)(void);

    Event(unsigned long interval, bool repeat, void (*callback)(void));

    bool check();
};

class EventScheduler {
  private:
    static const uint8_t maxEvents = 10;
    Event* events[maxEvents];
    uint8_t eventCount;

  public:
    EventScheduler();

    bool addEvent(unsigned long interval, bool repeat, void (*callback)(void));

    void run();
};

#endif // EVENT_SCHEDULER_H
