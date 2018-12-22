#ifndef __VIRTUAL_STEPPER_H
#define __VIRTUAL_STEPPER_H

#include <stdint.h> // uint8_t

struct virtual_stepper *virtual_stepper_oid_lookup(uint8_t oid);
uint8_t virtual_stepper_oid_verify(uint8_t oid);
uint8_t virtual_stepper_verify(void *stepper);
void virtual_stepper_stop(struct virtual_stepper *s);
uint32_t virtual_stepper_get_position(struct virtual_stepper *s);
void virtual_stepper_set_position(struct virtual_stepper *s, uint32_t position);

#endif // stepper.h
