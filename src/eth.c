/*
 * eth.c - ENC28J60 status broadcast and CUSTOM #1 upload
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LPC214x.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "t962.h"
#include "sched.h"
#include "sensor.h"
#include "reflow.h"
#include "reflow_profiles.h"
#include "eth.h"

/* GPIO SPI. Hardware SPI0/SSP pins are shared with the keypad, 1-wire and LCD. */
#define ETH_PIN_SCK  (1u << 17)
#define ETH_PIN_MOSI (1u << 6)
#define ETH_PIN_MISO (1u << 5)
#define ETH_PIN_CS   (1u << 10)
#define ETH_PIN_RST  (1u << 30)

/* Register address: bits 0-4 register, bits 5-6 bank, bit 7 = MAC/MII dummy read. */
#define ENC_ADDR_MASK 0x1f
#define ENC_BANK_MASK 0x60
#define ENC_SPRD      0x80

#define ENC_RCR 0x00
#define ENC_RBM 0x3a
#define ENC_WCR 0x40
#define ENC_WBM 0x7a
#define ENC_BFS 0x80
#define ENC_BFC 0xa0
#define ENC_SRC 0xff

#define EIE    0x1b
#define EIR    0x1c
#define ESTAT  0x1d
#define ECON2  0x1e
#define ECON1  0x1f

#define ERDPTL   (0x00)
#define EWRPTL   (0x02)
#define ETXSTL   (0x04)
#define ETXNDL   (0x06)
#define ERXSTL   (0x08)
#define ERXNDL   (0x0a)
#define ERXRDPTL (0x0c)

#define ERXFCON  (0x18 | 0x20)
#define EPKTCNT  (0x19 | 0x20)
#define ERXFCON_UCEN  0x80
#define ERXFCON_CRCEN 0x20
#define ERXFCON_BCEN  0x01

#define MACON1  (0x00 | 0x40 | ENC_SPRD)
#define MACON2  (0x01 | 0x40 | ENC_SPRD)
#define MACON3  (0x02 | 0x40 | ENC_SPRD)
#define MACON4  (0x03 | 0x40 | ENC_SPRD)
#define MABBIPG (0x04 | 0x40 | ENC_SPRD)
#define MAIPGL  (0x06 | 0x40 | ENC_SPRD)
#define MAMXFLL (0x0a | 0x40 | ENC_SPRD)
#define MICMD   (0x12 | 0x40 | ENC_SPRD)
#define MIREGADR (0x14 | 0x40 | ENC_SPRD)
#define MIWRL   (0x16 | 0x40 | ENC_SPRD)
#define MIRDL   (0x18 | 0x40 | ENC_SPRD)

/* Bank 3 MAC address bytes. MAADR5 is the first byte on the wire. */
#define MAADR1 (0x00 | 0x60 | ENC_SPRD)
#define MAADR0 (0x01 | 0x60 | ENC_SPRD)
#define MAADR3 (0x02 | 0x60 | ENC_SPRD)
#define MAADR2 (0x03 | 0x60 | ENC_SPRD)
#define MAADR5 (0x04 | 0x60 | ENC_SPRD)
#define MAADR4 (0x05 | 0x60 | ENC_SPRD)
#define MISTAT (0x0a | 0x60 | ENC_SPRD)
#define EREVID (0x12 | 0x60)

#define ESTAT_TXABRT 0x02
#define ESTAT_CLKRDY 0x01
#define EIR_TXIF     0x08
#define EIR_TXERIF   0x02
#define ECON2_PKTDEC 0x40
#define ECON1_TXRST  0x80
#define ECON1_RXRST  0x40
#define ECON1_TXRTS  0x08
#define ECON1_RXEN   0x04
#define ECON1_BSEL1  0x02
#define ECON1_BSEL0  0x01
#define MACON1_TXPAUS 0x08
#define MACON1_RXPAUS 0x04
#define MACON1_MARXEN 0x01
#define MACON3_PADCFG0 0x20
#define MACON3_TXCRCEN 0x10
#define MACON3_FRMLNEN 0x02
#define MACON4_DEFER   0x40
#define MICMD_MIIRD    0x01
#define MISTAT_BUSY    0x01

#define PHID1  0x02
#define PHCON2 0x10
#define PHSTAT2 0x11
#define PHLCON 0x14
#define PHCON2_HDLDIS 0x0100
#define PHSTAT2_LSTAT 0x0400
#define PHID_ENC28J60 0x0083

/* Errata: RX buffer must start at 0. ERXRDPT must be odd, so RXEND is odd. */
#define RXSTART 0x0000
#define RXEND   0x0fff
#define TXSTART 0x1000

#define ETHTYPE_IP  0x0800
#define ETHTYPE_ARP 0x0806
#define IP_PROTO_ICMP 1
#define IP_PROTO_UDP  17

#define ETH_TYPE_STATUS 1
#define ETH_TYPE_WRITE  2
#define ETH_TYPE_ACK    3

static const uint8_t our_mac[6] = { 0x02, 0x54, 0x39, 0x36, 0x32, 0x01 };
static const uint8_t our_ip[4] = { ETH_IP_A, ETH_IP_B, ETH_IP_C, ETH_IP_D };

static uint8_t enc_bank = 0xff;
static uint8_t eth_present = 0;
static uint8_t eth_rev = 0;
static uint8_t eth_link = 0;
static uint16_t rx_next = RXSTART;
static uint16_t ip_id = 1;
static uint8_t rxbuf[256];
static uint8_t txbuf[256];

static void spi_begin(void) {
	FIO0CLR = ETH_PIN_SCK;
	FIO0CLR = ETH_PIN_CS;
}

static void spi_end(void) {
	FIO0SET = ETH_PIN_CS;
}

static uint8_t spi_xfer(uint8_t out) {
	uint8_t in = 0;
	for (uint8_t bit = 0; bit < 8; bit++) {
		if (out & 0x80) {
			FIO0SET = ETH_PIN_MOSI;
		} else {
			FIO0CLR = ETH_PIN_MOSI;
		}
		out <<= 1;
		FIO0SET = ETH_PIN_SCK;
		in <<= 1;
		if (FIO0PIN & ETH_PIN_MISO) {
			in |= 1;
		}
		FIO0CLR = ETH_PIN_SCK;
	}
	return in;
}

static void enc_op(uint8_t op, uint8_t reg, uint8_t data) {
	spi_begin();
	spi_xfer(op | (reg & ENC_ADDR_MASK));
	spi_xfer(data);
	spi_end();
}

static void enc_bfs(uint8_t reg, uint8_t mask) {
	enc_op(ENC_BFS, reg, mask);
}

static void enc_bfc(uint8_t reg, uint8_t mask) {
	enc_op(ENC_BFC, reg, mask);
}

static void enc_set_bank(uint8_t reg) {
	uint8_t bank;
	/* EIR/ESTAT/ECON1/ECON2/EIE are present in every bank. */
	if ((reg & ENC_ADDR_MASK) >= 0x1b) {
		return;
	}
	bank = reg & ENC_BANK_MASK;
	if (bank == enc_bank) {
		return;
	}
	enc_bfc(ECON1, ECON1_BSEL1 | ECON1_BSEL0);
	enc_bfs(ECON1, bank >> 5);
	enc_bank = bank;
}

static uint8_t enc_read(uint8_t reg) {
	uint8_t value;
	enc_set_bank(reg);
	spi_begin();
	spi_xfer(ENC_RCR | (reg & ENC_ADDR_MASK));
	if (reg & ENC_SPRD) {
		spi_xfer(0);
	}
	value = spi_xfer(0);
	spi_end();
	return value;
}

static void enc_write(uint8_t reg, uint8_t value) {
	enc_set_bank(reg);
	enc_op(ENC_WCR, reg, value);
}

static uint16_t enc_r16(uint8_t reg) {
	return enc_read(reg) | ((uint16_t)enc_read(reg + 1) << 8);
}

static void enc_w16(uint8_t reg, uint16_t value) {
	enc_write(reg, (uint8_t)value);
	enc_write(reg + 1, (uint8_t)(value >> 8));
}

static void rbm_read(uint8_t* dst, uint16_t len) {
	spi_begin();
	spi_xfer(ENC_RBM);
	for (uint16_t i = 0; i < len; i++) {
		uint8_t value = spi_xfer(0);
		if (dst) {
			dst[i] = value;
		}
	}
	spi_end();
}

static int mii_wait(void) {
	for (uint16_t i = 0; i < 1000; i++) {
		if ((enc_read(MISTAT) & MISTAT_BUSY) == 0) {
			return 0;
		}
	}
	return -1;
}

static uint16_t phy_read(uint8_t reg) {
	enc_write(MIREGADR, reg);
	enc_write(MICMD, MICMD_MIIRD);
	if (mii_wait()) {
		enc_write(MICMD, 0);
		return 0xffff;
	}
	enc_write(MICMD, 0);
	return enc_r16(MIRDL);
}

static int phy_write(uint8_t reg, uint16_t value) {
	enc_write(MIREGADR, reg);
	enc_w16(MIWRL, value);
	return mii_wait();
}

static void enc_rx_reset(void) {
	uint8_t guard = 0;
	enc_bfc(ECON1, ECON1_RXEN);
	enc_bfs(ECON1, ECON1_RXRST);
	enc_bfc(ECON1, ECON1_RXRST);
	rx_next = RXSTART;
	enc_w16(ERXSTL, RXSTART);
	enc_w16(ERXNDL, RXEND);
	enc_w16(ERXRDPTL, RXEND);
	while (enc_read(EPKTCNT) != 0 && guard < 255) {
		enc_bfs(ECON2, ECON2_PKTDEC);
		guard++;
	}
	enc_bfs(ECON1, ECON1_RXEN);
}

static int enc_tx(const uint8_t* frame, uint16_t len) {
	uint8_t idle = 0;
	if (len == 0 || len > 1514) {
		return -1;
	}
	for (uint16_t i = 0; i < 1000; i++) {
		if ((enc_read(ECON1) & ECON1_TXRTS) == 0) {
			idle = 1;
			break;
		}
		if (enc_read(EIR) & EIR_TXERIF) {
			break;
		}
	}
	if (!idle) {
		enc_bfs(ECON1, ECON1_TXRST);
		enc_bfc(ECON1, ECON1_TXRST);
	}
	enc_bfc(EIR, EIR_TXIF | EIR_TXERIF);

	/* Control byte at TXSTART, then the frame. ETXND is the last byte. */
	enc_w16(EWRPTL, TXSTART);
	enc_w16(ETXNDL, TXSTART + len);
	spi_begin();
	spi_xfer(ENC_WBM);
	spi_xfer(0x00);
	for (uint16_t i = 0; i < len; i++) {
		spi_xfer(frame[i]);
	}
	spi_end();
	enc_bfs(ECON1, ECON1_TXRTS);
	return 0;
}

/* Returns the frame length, 0 when the FIFO is empty, or -1 if the packet was dropped. */
static int enc_rx(uint8_t* buf, uint16_t buflen) {
	uint8_t hdr[6];
	uint16_t next;
	uint16_t count;
	uint16_t status;
	uint16_t framelen = 0;
	uint16_t erxrdpt;

	if (enc_read(EPKTCNT) == 0) {
		return 0;
	}
	enc_w16(ERDPTL, rx_next);
	rbm_read(hdr, sizeof(hdr));
	next = hdr[0] | ((uint16_t)hdr[1] << 8);
	count = hdr[2] | ((uint16_t)hdr[3] << 8);
	status = hdr[4] | ((uint16_t)hdr[5] << 8);

	if (next > RXEND) {
		enc_rx_reset();
		return -1;
	}
	if ((status & 0x80) && count >= 18 && count <= 1518) {
		framelen = count - 4; /* Drop the CRC. */
		if (framelen > buflen) {
			framelen = 0;
		}
	}
	if (framelen) {
		rbm_read(buf, framelen);
	}

	rx_next = next;
	/* Errata: only odd addresses may be written to ERXRDPT. A zero next
	 * pointer wraps this subtraction past RXEND, which selects RXEND. */
	erxrdpt = (uint16_t)(rx_next - 1);
	if ((erxrdpt & 1u) == 0) {
		erxrdpt--;
	}
	if (erxrdpt > RXEND) {
		erxrdpt = RXEND;
	}
	enc_w16(ERXRDPTL, erxrdpt);
	enc_bfs(ECON2, ECON2_PKTDEC);
	return framelen ? (int)framelen : -1;
}

static int enc_link_up(void) {
	(void)phy_read(PHSTAT2);
	return (phy_read(PHSTAT2) & PHSTAT2_LSTAT) != 0;
}

static int enc_init(void) {
	uint16_t phid;
	FIO0CLR = ETH_PIN_RST;
	BusyWait(TICKS_MS(1));
	FIO0SET = ETH_PIN_RST;
	BusyWait(TICKS_MS(2));

	enc_bank = 0xff;
	spi_begin();
	spi_xfer(ENC_SRC);
	spi_end();
	BusyWait(TICKS_MS(2)); /* Errata: no SPI for 1 ms after a soft reset. */
	enc_bfc(ECON1, ECON1_BSEL1 | ECON1_BSEL0);
	enc_bank = 0;

	for (uint8_t i = 0; i < 20; i++) {
		if (enc_read(ESTAT) & ESTAT_CLKRDY) {
			break;
		}
		BusyWait(TICKS_MS(1));
		if (i == 19) {
			return -1;
		}
	}

	rx_next = RXSTART;
	enc_w16(ERXSTL, RXSTART);
	enc_w16(ERXNDL, RXEND);
	enc_w16(ERXRDPTL, RXEND);
	enc_w16(ETXSTL, TXSTART);

	enc_write(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);
	enc_write(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	enc_write(MACON2, 0x00);
	enc_write(MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
	enc_write(MACON4, MACON4_DEFER);
	enc_w16(MAMXFLL, 1518);
	enc_write(MABBIPG, 0x12); /* Half duplex. */
	enc_w16(MAIPGL, 0x0c12);

	enc_write(MAADR5, our_mac[0]);
	enc_write(MAADR4, our_mac[1]);
	enc_write(MAADR3, our_mac[2]);
	enc_write(MAADR2, our_mac[3]);
	enc_write(MAADR1, our_mac[4]);
	enc_write(MAADR0, our_mac[5]);

	if (phy_write(PHCON2, PHCON2_HDLDIS)) {
		return -1;
	}
	phy_write(PHLCON, 0x0476);

	phid = phy_read(PHID1);
	eth_rev = enc_read(EREVID);
	if (phid != PHID_ENC28J60) {
		return -1;
	}
	enc_bfs(ECON1, ECON1_RXEN);
	return 0;
}

static uint16_t rd16(const uint8_t* p) {
	return ((uint16_t)p[0] << 8) | p[1];
}

static void wr16(uint8_t* p, uint16_t value) {
	p[0] = (uint8_t)(value >> 8);
	p[1] = (uint8_t)value;
}

static uint32_t csum_add(uint32_t sum, const uint8_t* data, int len) {
	while (len > 1) {
		sum += ((uint32_t)data[0] << 8) | data[1];
		data += 2;
		len -= 2;
	}
	if (len > 0) {
		sum += (uint32_t)data[0] << 8;
	}
	return sum;
}

static uint16_t csum_fold(uint32_t sum) {
	while (sum >> 16) {
		sum = (sum & 0xffffu) + (sum >> 16);
	}
	return (uint16_t)sum;
}

static uint16_t csum_finish(uint32_t sum) {
	return (uint16_t)~csum_fold(sum);
}

static int csum_ok(const uint8_t* data, int len, uint32_t sum) {
	return csum_fold(csum_add(sum, data, len)) == 0xffff;
}

static void send_udp(const uint8_t dst_mac[6], const uint8_t dst_ip[4],
		uint16_t dst_port, const uint8_t* payload, uint16_t plen) {
	uint16_t ulen = 8 + plen;
	uint16_t iplen = 20 + ulen;
	uint8_t* ip;
	if ((uint32_t)14 + iplen > sizeof(txbuf)) {
		return;
	}
	memcpy(txbuf, dst_mac, 6);
	memcpy(txbuf + 6, our_mac, 6);
	wr16(txbuf + 12, ETHTYPE_IP);
	ip = txbuf + 14;
	memset(ip, 0, 20);
	ip[0] = 0x45;
	wr16(ip + 2, iplen);
	wr16(ip + 4, ip_id++);
	ip[8] = 64;
	ip[9] = IP_PROTO_UDP;
	memcpy(ip + 12, our_ip, 4);
	memcpy(ip + 16, dst_ip, 4);
	wr16(ip + 10, csum_finish(csum_add(0, ip, 20)));
	wr16(ip + 20, ETH_UDP_PORT);
	wr16(ip + 22, dst_port);
	wr16(ip + 24, ulen);
	wr16(ip + 26, 0); /* IPv4 allows a zero UDP checksum. */
	memcpy(ip + 28, payload, plen);
	enc_tx(txbuf, 14 + iplen);
}

static void send_status(void) {
	uint8_t payload[34];
	float temp;
	int16_t temp_x10;
	uint16_t setpoint;
	const char* name;
	static const uint8_t bcast_mac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	static const uint8_t bcast_ip[4] = { 255, 255, 255, 255 };

	memset(payload, 0, sizeof(payload));
	payload[0] = 'T';
	payload[1] = '9';
	payload[2] = '6';
	payload[3] = '2';
	payload[4] = ETH_TYPE_STATUS;
	payload[5] = (uint8_t)Reflow_GetMode();
	payload[6] = (uint8_t)Reflow_GetProfileIdx();
	payload[7] = Reflow_GetFan();
	payload[8] = Reflow_GetHeat();
	temp = Sensor_GetTemp(TC_AVERAGE);
	if (temp >= 0) {
		temp_x10 = (int16_t)(temp * 10.0f + 0.5f);
	} else {
		temp_x10 = (int16_t)(temp * 10.0f - 0.5f);
	}
	payload[10] = (uint8_t)temp_x10;
	payload[11] = (uint8_t)(temp_x10 >> 8);
	setpoint = Reflow_GetSetpoint();
	payload[12] = (uint8_t)setpoint;
	payload[13] = (uint8_t)(setpoint >> 8);
	name = Reflow_GetProfileName();
	for (int i = 0; i < 19 && name[i]; i++) {
		payload[14 + i] = (uint8_t)name[i];
	}
	send_udp(bcast_mac, bcast_ip, ETH_UDP_PORT, payload, sizeof(payload));
}

static int ip_for_us(const uint8_t* ip) {
	static const uint8_t bcast[4] = { 255, 255, 255, 255 };
	if (memcmp(ip, our_ip, 4) == 0 || memcmp(ip, bcast, 4) == 0) {
		return 1;
	}
	return 0;
}

static void handle_write(const uint8_t* ip, uint16_t sport, const uint8_t* payload, uint16_t plen) {
	uint8_t status = 1;
	uint8_t ack[6];
	if (plen >= 6 + (NUMPROFILETEMPS * 2) && payload[5] == NUMPROFILETEMPS) {
		uint16_t temps[NUMPROFILETEMPS];
		int rc;
		for (int i = 0; i < NUMPROFILETEMPS; i++) {
			uint16_t off = 6 + (i * 2);
			temps[i] = payload[off] | ((uint16_t)payload[off + 1] << 8);
		}
		rc = Reflow_WriteCustom1(temps);
		if (rc == 0) {
			status = 0;
			printf("\nCUSTOM #1 stored from ethernet");
			if (Reflow_GetMode() != REFLOW_REFLOW) {
				Reflow_SelectCustom1();
			}
		} else if (rc == -1) {
			status = 1;
			printf("\nCUSTOM #1 ethernet write rejected");
		} else {
			status = 2;
			printf("\nCUSTOM #1 ethernet write failed");
		}
	} else {
		printf("\nCUSTOM #1 ethernet write rejected");
	}
	ack[0] = 'T';
	ack[1] = '9';
	ack[2] = '6';
	ack[3] = '2';
	ack[4] = ETH_TYPE_ACK;
	ack[5] = status;
	send_udp(rxbuf + 6, ip + 12, sport, ack, sizeof(ack));
}

static void handle_arp(uint16_t len) {
	const uint8_t* arp;
	if (len < 42) {
		return;
	}
	arp = rxbuf + 14;
	if (rd16(arp) != 1 || rd16(arp + 2) != ETHTYPE_IP) {
		return;
	}
	if (arp[4] != 6 || arp[5] != 4 || rd16(arp + 6) != 1) {
		return;
	}
	if (memcmp(arp + 24, our_ip, 4) != 0) {
		return;
	}
	memcpy(txbuf, arp + 8, 6);
	memcpy(txbuf + 6, our_mac, 6);
	wr16(txbuf + 12, ETHTYPE_ARP);
	wr16(txbuf + 14, 1);
	wr16(txbuf + 16, ETHTYPE_IP);
	txbuf[18] = 6;
	txbuf[19] = 4;
	wr16(txbuf + 20, 2);
	memcpy(txbuf + 22, our_mac, 6);
	memcpy(txbuf + 28, our_ip, 4);
	memcpy(txbuf + 32, arp + 8, 6);
	memcpy(txbuf + 38, arp + 14, 4);
	enc_tx(txbuf, 42);
}

static void handle_icmp(const uint8_t* ip, uint16_t iplen) {
	uint8_t* reply;
	if (iplen < 28 || ip[20] != 8 || ip[21] != 0) {
		return;
	}
	if (memcmp(ip + 16, our_ip, 4) != 0) {
		return;
	}
	if ((uint32_t)14 + iplen > sizeof(txbuf)) {
		return;
	}
	memcpy(txbuf, rxbuf + 6, 6);
	memcpy(txbuf + 6, our_mac, 6);
	wr16(txbuf + 12, ETHTYPE_IP);
	reply = txbuf + 14;
	memcpy(reply, ip, iplen);
	memcpy(reply + 12, our_ip, 4);
	memcpy(reply + 16, ip + 12, 4);
	reply[8] = 64;
	reply[10] = 0;
	reply[11] = 0;
	wr16(reply + 10, csum_finish(csum_add(0, reply, 20)));
	reply[20] = 0;
	reply[22] = 0;
	reply[23] = 0;
	wr16(reply + 22, csum_finish(csum_add(0, reply + 20, iplen - 20)));
	enc_tx(txbuf, 14 + iplen);
}

static void handle_udp(const uint8_t* ip, uint16_t iplen) {
	const uint8_t* udp;
	const uint8_t* payload;
	uint16_t sport;
	uint16_t dport;
	uint16_t ulen;
	uint16_t csum;
	uint16_t plen;
	uint32_t sum;
	if (iplen < 28) {
		return;
	}
	udp = ip + 20;
	sport = rd16(udp);
	dport = rd16(udp + 2);
	ulen = rd16(udp + 4);
	csum = rd16(udp + 6);
	if (dport != ETH_UDP_PORT || ulen < 8 || (uint32_t)20 + ulen > iplen) {
		return;
	}
	if (csum != 0) {
		sum = 0;
		sum += ((uint32_t)ip[12] << 8) | ip[13];
		sum += ((uint32_t)ip[14] << 8) | ip[15];
		sum += ((uint32_t)ip[16] << 8) | ip[17];
		sum += ((uint32_t)ip[18] << 8) | ip[19];
		sum += IP_PROTO_UDP;
		sum += ulen;
		if (!csum_ok(udp, ulen, sum)) {
			return;
		}
	}
	payload = udp + 8;
	plen = ulen - 8;
	if (plen < 5 || memcmp(payload, "T962", 4) != 0) {
		return;
	}
	if (payload[4] == ETH_TYPE_WRITE) {
		handle_write(ip, sport, payload, plen);
	}
}

static void handle_ip(uint16_t len) {
	const uint8_t* ip;
	uint16_t iplen;
	if (len < 34) {
		return;
	}
	ip = rxbuf + 14;
	if (ip[0] != 0x45) {
		return;
	}
	iplen = rd16(ip + 2);
	if (iplen < 20 || (uint32_t)14 + iplen > len) {
		return;
	}
	if (!csum_ok(ip, 20, 0)) {
		return;
	}
	if (!ip_for_us(ip + 16)) {
		return;
	}
	if (ip[9] == IP_PROTO_ICMP) {
		handle_icmp(ip, iplen);
	} else if (ip[9] == IP_PROTO_UDP) {
		handle_udp(ip, iplen);
	}
}

static void handle_frame(uint16_t len) {
	uint16_t ethertype;
	if (len < 14) {
		return;
	}
	if (memcmp(rxbuf + 6, our_mac, 6) == 0) {
		return;
	}
	ethertype = rd16(rxbuf + 12);
	if (ethertype == ETHTYPE_ARP) {
		handle_arp(len);
	} else if (ethertype == ETHTYPE_IP) {
		handle_ip(len);
	}
}

static int32_t Eth_Work(void) {
	static uint32_t last = 0;
	uint32_t now = Sched_GetTick();
	for (int i = 0; i < 4; i++) {
		int n = enc_rx(rxbuf, sizeof(rxbuf));
		if (n == 0) {
			break;
		}
		if (n > 0) {
			handle_frame((uint16_t)n);
		}
	}
	if ((now - last) >= TICKS_SECS(1)) {
		last = now;
		uint8_t up = enc_link_up() ? 1 : 0;
		if (up != eth_link) {
			eth_link = up;
			printf("\nENC28J60 link %s", up ? "up" : "down");
		}
		if (up) {
			send_status();
		}
	}
	return TICKS_MS(100);
}

static void spi_pins_init(void) {
	FIO0MASK &= ~(ETH_PIN_SCK | ETH_PIN_MOSI | ETH_PIN_MISO | ETH_PIN_CS | ETH_PIN_RST);
	FIO0DIR |= ETH_PIN_SCK | ETH_PIN_MOSI | ETH_PIN_CS | ETH_PIN_RST;
	FIO0DIR &= ~ETH_PIN_MISO;
	FIO0SET = ETH_PIN_CS | ETH_PIN_RST;
	FIO0CLR = ETH_PIN_SCK | ETH_PIN_MOSI;
}

void Eth_Init(void) {
	spi_pins_init();
	if (enc_init() != 0) {
		eth_present = 0;
		printf("\nENC28J60 not found (SCK P0.17 MOSI P0.6 MISO P0.5 CS P0.10 RESET P0.30)");
		return;
	}
	eth_present = 1;
	eth_link = enc_link_up() ? 1 : 0;
	printf("\nENC28J60 rev %u %s, IP %u.%u.%u.%u UDP %u",
		(unsigned)eth_rev,
		eth_link ? "link up" : "link down",
		ETH_IP_A, ETH_IP_B, ETH_IP_C, ETH_IP_D, ETH_UDP_PORT);
	Sched_SetWorkfunc(ETH_WORK, Eth_Work);
	Sched_SetState(ETH_WORK, 2, 0);
}

void Eth_PrintStatus(void) {
	if (!eth_present) {
		printf("\nENC28J60 not detected");
		printf("\nSPI SCK P0.17 MOSI P0.6 MISO P0.5 CS P0.10 RESET P0.30\n");
		return;
	}
	eth_link = enc_link_up() ? 1 : 0;
	printf("\nENC28J60 rev %u, link %s", (unsigned)eth_rev, eth_link ? "up" : "down");
	printf("\nMAC %02x:%02x:%02x:%02x:%02x:%02x",
		(unsigned)our_mac[0], (unsigned)our_mac[1], (unsigned)our_mac[2],
		(unsigned)our_mac[3], (unsigned)our_mac[4], (unsigned)our_mac[5]);
	printf("\nIP %u.%u.%u.%u UDP %u (status broadcast, CUSTOM #1 write)\n",
		ETH_IP_A, ETH_IP_B, ETH_IP_C, ETH_IP_D, ETH_UDP_PORT);
}
