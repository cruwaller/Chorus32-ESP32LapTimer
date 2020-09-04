#ifndef _ESPNOW_H_
#define _ESPNOW_H_

#include <stdint.h>
#include <stddef.h>


void espnow_init(uint8_t if_type);

void espnow_sendPacket(void* output, uint8_t* data, size_t len);
void espnow_processPacket(void* output);

#endif /* _ESPNOW_H_ */
