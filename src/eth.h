#ifndef ETH_H_
#define ETH_H_

/*
 * ENC28J60 on a GPIO SPI bus. The LPC2134/LPC214x hardware SPI and SSP
 * pins are already used, so the wiring (3.3 V logic) is:
 *   P0.17 SCK, P0.6 MOSI, P0.5 MISO, P0.10 CS (active low), P0.30 RESET (active low)
 *
 * Address 192.168.0.62, UDP port 1962. Payload multi-byte fields are
 * little-endian. While the link is up the oven broadcasts one status
 * datagram per second to 255.255.255.255:1962. A host writes CUSTOM #1 by
 * sending a datagram to 192.168.0.62:1962. The oven answers ARP and ping.
 *
 * Status, type 1, 34 bytes:
 *   0  char magic[4]         'T','9','6','2'
 *   4  uint8 type            1
 *   5  uint8 mode            ReflowMode_t
 *   6  uint8 profile_idx
 *   7  uint8 fan             convection fan PWM, 0-255
 *   8  uint8 heat            heater PWM, 0-255
 *   9  uint8 reserved
 *  10  int16 temp_x10        average oven temperature in tenths of a degree C
 *  12  uint16 setpoint
 *  14  char profile_name[20] zero padded
 *
 * Write CUSTOM #1, type 2, 102 bytes:
 *   0  char magic[4]
 *   4  uint8 type            2
 *   5  uint8 count           48
 *   6  uint16 temps[48]      degrees C, each 0 or 30..300
 *
 * Write result, type 3, 6 bytes, unicast back to the sender:
 *   0  char magic[4]
 *   4  uint8 type            3
 *   5  uint8 status          0 saved, 1 rejected, 2 EEPROM error
 *
 * A stored profile replaces CUSTOM #1 in RAM as well. It becomes the
 * selected profile unless a reflow is already running.
 */

#define ETH_IP_A 192
#define ETH_IP_B 168
#define ETH_IP_C 0
#define ETH_IP_D 62
#define ETH_UDP_PORT 1962

void Eth_Init(void);
void Eth_PrintStatus(void);

#endif /* ETH_H_ */
