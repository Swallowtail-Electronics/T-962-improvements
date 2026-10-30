#include "LPC214x.h"
#include <stdint.h>
#include <stdio.h>
#include "t962.h"
#include "lcd.h"
#include "nvstorage.h"
#include "eeprom.h"
#include "reflow.h"

#include "reflow_profiles.h"

#define RAMPTEST
#define PIDTEST

extern uint8_t graphbmp[];

// Amtech 4300 63Sn/37Pb leaded profile
static const profile am4300profile = {
	"4300 63SN/37PB", {
		 50, 50, 50, 60, 73, 86,100,113,126,140,143,147,150,154,157,161, // 0-150s
		164,168,171,175,179,183,195,207,215,207,195,183,168,154,140,125, // Adjust peak from 205 to 220C
		111, 97, 82, 68, 54,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};

// NC-31 low-temp lead-free profile
static const profile nc31profile = {
	"NC-31 LOW-TEMP LF", {
		 50, 50, 50, 50, 55, 70, 85, 90, 95,100,102,105,107,110,112,115, // 0-150s
		117,120,122,127,132,138,148,158,160,158,148,138,130,122,114,106, // Adjust peak from 158 to 165C
		 98, 90, 82, 74, 66, 58,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};

// SynTECH-LF normal temp lead-free profile
static const profile syntechlfprofile = {
	"AMTECH SYNTECH-LF", {
		 50, 50, 50, 50, 60, 70, 80, 90,100,110,120,130,140,149,158,166, // 0-150s
		175,184,193,201,210,219,230,240,245,240,230,219,212,205,198,191, // Adjust peak from 230 to 249C
		184,177,157,137,117, 97, 77, 57,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};

#ifdef RAMPTEST
// Ramp speed test temp profile
static const profile rampspeed_testprofile = {
	"RAMP SPEED TEST", {
		 50, 50, 50, 50,245,245,245,245,245,245,245,245,245,245,245,245, // 0-150s
		245,245,245,245,245,245,245,245,245, 50, 50, 50, 50, 50, 50, 50, // 160-310s
		 50, 50, 50, 50, 50, 50, 50, 50,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};
#endif

#ifdef PIDTEST
// PID gain adjustment test profile (5% setpoint change)
static const profile pidcontrol_testprofile = {
	"PID CONTROL TEST",	{
		171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171, // 0-150s
		180,180,180,180,180,180,180,180,171,171,171,171,171,171,171,171, // 160-310s
		  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};
#endif

// EEPROM profile 1
static ramprofile ee1 = { "CUSTOM #1" };

// EEPROM profile 2
static ramprofile ee2 = { "CUSTOM #2" };

static const profile* profiles[] = {
	&syntechlfprofile,
	&nc31profile,
	&am4300profile,
#ifdef RAMPTEST
	&rampspeed_testprofile,
#endif
#ifdef PIDTEST
	&pidcontrol_testprofile,
#endif
	(profile*) &ee1,
	(profile*) &ee2
};

#define NUMPROFILES (sizeof(profiles) / sizeof(profiles[0]))

// current profile index
static uint8_t profileidx = 0;

static void ByteswapTempProfile(uint16_t* buf) {
	for (int i = 0; i < NUMPROFILETEMPS; i++) {
		uint16_t word = buf[i];
		buf[i] = word >> 8 | word << 8;
	}
}

void Reflow_LoadCustomProfiles(void) {
	EEPROM_Read((uint8_t*)ee1.temperatures, 2, 96);
	ByteswapTempProfile(ee1.temperatures);

	EEPROM_Read((uint8_t*)ee2.temperatures, 128 + 2, 96);
	ByteswapTempProfile(ee2.temperatures);
}

void Reflow_ValidateNV(void) {
	if (NV_GetConfig(REFLOW_BEEP_DONE_LEN) == 255) {
		// Default 1 second beep length
		NV_SetConfig(REFLOW_BEEP_DONE_LEN, 10);
	}

	if (NV_GetConfig(REFLOW_MIN_FAN_SPEED) == 255) {
		// Default fan speed is now 8
		NV_SetConfig(REFLOW_MIN_FAN_SPEED, 8);
	}

	if (NV_GetConfig(REFLOW_BAKE_SETPOINT_H) == 255 || NV_GetConfig(REFLOW_BAKE_SETPOINT_L) == 255) {
		NV_SetConfig(REFLOW_BAKE_SETPOINT_H, SETPOINT_DEFAULT >> 8);
		NV_SetConfig(REFLOW_BAKE_SETPOINT_L, (uint8_t)SETPOINT_DEFAULT);
		printf("Resetting bake setpoint to default.");
	}

	Reflow_SelectProfileIdx(NV_GetConfig(REFLOW_PROFILE));
}

int Reflow_GetProfileIdx(void) {
	return profileidx;
}

int Reflow_SelectProfileIdx(int idx) {
	if (idx < 0) {
		profileidx = (NUMPROFILES - 1);
	} else if(idx >= NUMPROFILES) {
		profileidx = 0;
	} else {
		profileidx = idx;
	}
	NV_SetConfig(REFLOW_PROFILE, profileidx);
	return profileidx;
}

int Reflow_SelectEEProfileIdx(int idx) {
	if (idx == 1) {
		profileidx = (NUMPROFILES - 2);
	} else if (idx == 2) {
		profileidx = (NUMPROFILES - 1);
	}
	return profileidx;
}

int Reflow_GetEEProfileIdx(void) {
	if (profileidx == (NUMPROFILES - 2)) {
		return 1;
	} else 	if (profileidx == (NUMPROFILES - 1)) {
		return 2;
	} else {
		return 0;
	}
}

int Reflow_SaveEEProfile(void) {
	int retval = 0;
	uint8_t offset;
	uint16_t* tempptr;
	if (profileidx == (NUMPROFILES - 2)) {
		offset = 0;
		tempptr = ee1.temperatures;
	} else if (profileidx == (NUMPROFILES - 1)) {
		offset = 128;
		tempptr = ee2.temperatures;
	} else {
		return -1;
	}
	offset += 2; // Skip "magic"
	ByteswapTempProfile(tempptr);

	// Store profile
	retval = EEPROM_Write(offset, (uint8_t*)tempptr, 96);
	ByteswapTempProfile(tempptr);
	return retval;
}

// CUSTOM #1 EEPROM image is 48 big-endian words at address 2.
#define CUSTOM1_BYTES (NUMPROFILETEMPS * 2)

enum {
	C1_IDLE = 0,
	C1_DATA,
	C1_MAGIC_LO,
	C1_MAGIC_HI,
	C1_REVERT,
	C1_REVERT_LO,
	C1_REVERT_HI
};

static uint8_t c1_phase = C1_IDLE;
static uint8_t c1_pos = 0;
static uint8_t c1_fail = 0;
static uint8_t c1_bytes[CUSTOM1_BYTES];

// One EEPROM page is 8 bytes. A page program is about 5 ms, so the caller
// writes a single page and then returns to the scheduler.
static int write_one_page(uint8_t addr, uint8_t* src, uint8_t left) {
	uint8_t room = (uint8_t)(8u - (addr & 7u));
	uint8_t n = left < room ? left : room;
	if (n == 0 || EEPROM_Write(addr, src, n)) {
		return -1;
	}
	return n;
}

static void begin_revert(void) {
	c1_pos = 0;
	c1_phase = C1_REVERT;
}

// Previous curve, as currently running in RAM, in EEPROM byte order.
static void previous_page(uint8_t* dst, uint8_t pos, uint8_t n) {
	for (uint8_t i = 0; i < n; i++) {
		uint8_t index = (uint8_t)(pos + i);
		uint16_t value = ee1.temperatures[index >> 1];
		dst[i] = (index & 1u) ? (uint8_t)value : (uint8_t)(value >> 8);
	}
}

int Reflow_WriteCustom1(const uint16_t* temps) {
	if (!temps || c1_phase != C1_IDLE) {
		return -1;
	}
	// Bake, reflow, and standby-fan keep the curve they started with.
	if (Reflow_GetMode() != REFLOW_STANDBY) {
		return -1;
	}
	for (int i = 0; i < NUMPROFILETEMPS; i++) {
		uint16_t value = temps[i];
		// 0 ends the curve. Any other point has to be a legal setpoint.
		if (value > SETPOINT_MAX || (value != 0 && value < SETPOINT_MIN)) {
			return -1;
		}
		c1_bytes[i * 2] = (uint8_t)(value >> 8);
		c1_bytes[i * 2 + 1] = (uint8_t)value;
	}
	c1_pos = 0;
	c1_fail = 0;
	c1_phase = C1_DATA;
	return 0;
}

int Reflow_ServiceCustom1(void) {
	uint8_t scratch[8];
	int n;

	if (c1_phase == C1_IDLE) {
		return -3;
	}

	// Stop before the first byte when the oven has left standby. After any
	// byte has been programmed, finish by restoring the RAM curve to EEPROM
	// so a torn image is not what the next boot loads.
	if (c1_phase == C1_DATA && Reflow_GetMode() != REFLOW_STANDBY) {
		if (c1_pos == 0) {
			c1_phase = C1_IDLE;
			return -1;
		}
		begin_revert();
		return 1;
	}

	if (c1_phase == C1_DATA) {
		n = write_one_page((uint8_t)(2 + c1_pos), c1_bytes + c1_pos, (uint8_t)(CUSTOM1_BYTES - c1_pos));
		if (n < 0) {
			c1_fail = 1;
			begin_revert();
			return 1;
		}
		c1_pos = (uint8_t)(c1_pos + n);
		if (c1_pos >= CUSTOM1_BYTES) {
			c1_phase = C1_MAGIC_LO;
		}
		return 1;
	}

	if (c1_phase == C1_MAGIC_LO) {
		scratch[0] = 0x56;
		scratch[1] = 0x57;
		if (EEPROM_Write(0, scratch, 2)) {
			c1_fail = 1;
			begin_revert();
			return 1;
		}
		c1_phase = C1_MAGIC_HI;
		return 1;
	}

	if (c1_phase == C1_MAGIC_HI) {
		scratch[0] = 0x58;
		scratch[1] = 0x59;
		if (EEPROM_Write(0x7e, scratch, 2)) {
			c1_fail = 1;
			begin_revert();
			return 1;
		}
		// The live curve changes only while standby still holds. The EEPROM
		// image is restored below when a run started during the commit.
		if (Reflow_GetMode() == REFLOW_STANDBY) {
			for (int i = 0; i < NUMPROFILETEMPS; i++) {
				ee1.temperatures[i] = ((uint16_t)c1_bytes[i * 2] << 8) | c1_bytes[i * 2 + 1];
			}
			Reflow_SelectCustom1();
			c1_phase = C1_IDLE;
			return 0;
		}
		begin_revert();
		return 1;
	}

	if (c1_phase == C1_REVERT) {
		uint8_t left = (uint8_t)(CUSTOM1_BYTES - c1_pos);
		uint8_t count = (uint8_t)(8u - ((2 + c1_pos) & 7u));
		if (count > left) {
			count = left;
		}
		previous_page(scratch, c1_pos, count);
		if (EEPROM_Write((uint8_t)(2 + c1_pos), scratch, count)) {
			c1_phase = C1_IDLE;
			return -2;
		}
		c1_pos = (uint8_t)(c1_pos + count);
		if (c1_pos >= CUSTOM1_BYTES) {
			c1_phase = C1_REVERT_LO;
		}
		return 1;
	}

	if (c1_phase == C1_REVERT_LO) {
		scratch[0] = 0x56;
		scratch[1] = 0x57;
		if (EEPROM_Write(0, scratch, 2)) {
			c1_phase = C1_IDLE;
			return -2;
		}
		c1_phase = C1_REVERT_HI;
		return 1;
	}

	// C1_REVERT_HI. RAM was never updated, and EEPROM matches it again.
	scratch[0] = 0x58;
	scratch[1] = 0x59;
	c1_phase = C1_IDLE;
	if (EEPROM_Write(0x7e, scratch, 2)) {
		return -2;
	}
	return c1_fail ? -2 : -1;
}

int Reflow_SelectCustom1(void) {
	return Reflow_SelectProfileIdx(NUMPROFILES - 2);
}

void Reflow_ListProfiles(void) {
	for (int i = 0; i < NUMPROFILES; i++) {
		printf("%d: %s\n", i, profiles[i]->name);
	}
}

const char* Reflow_GetProfileName(void) {
	return profiles[profileidx]->name;
}

uint16_t Reflow_GetSetpointAtIdx(uint8_t idx) {
	if (idx > (NUMPROFILETEMPS - 1)) {
		return 0;
	}
	return profiles[profileidx]->temperatures[idx];
}

void Reflow_SetSetpointAtIdx(uint8_t idx, uint16_t value) {
	if (idx > (NUMPROFILETEMPS - 1)) { return; }
	if (value > SETPOINT_MAX) { return; }

	uint16_t* temp = (uint16_t*) &profiles[profileidx]->temperatures[idx];
	if (temp >= (uint16_t*)0x40000000) {
		*temp = value; // If RAM-based
	}
}

void Reflow_PlotProfile(int highlight) {
	LCD_BMPDisplay(graphbmp, 0, 0);

	// No need to plot first value as it is obscured by Y-axis
	for(int x = 1; x < NUMPROFILETEMPS; x++) {
		int realx = (x << 1) + XAXIS;
		int y = profiles[profileidx]->temperatures[x] / 5;
		y = YAXIS - y;
		LCD_SetPixel(realx, y);

		if (highlight == x) {
			LCD_SetPixel(realx - 1, y - 1);
			LCD_SetPixel(realx + 1, y + 1);
			LCD_SetPixel(realx - 1, y + 1);
			LCD_SetPixel(realx + 1, y - 1);
		}
	}
}

void Reflow_DumpProfile(int profile) {
	if (profile > NUMPROFILES) {
		printf("\nNo profile with id: %d\n", profile);
		return;
	}

	int current = profileidx;
	profileidx = profile;

	for (int i = 0; i < NUMPROFILETEMPS; i++) {
		printf("%4d,", Reflow_GetSetpointAtIdx(i));
		if (i == 15 || i == 31) {
			printf("\n ");
		}
	}
	printf("\n");
	profileidx = current;
}
