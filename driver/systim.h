#ifndef SYSTIM_H
#define SYSTIM_H

typedef struct {
    uint32_t tag;
    uint32_t period;
} TimeTag;

uint8_t check_timetag(TimeTag* timetag);
#endif