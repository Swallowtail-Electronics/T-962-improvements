#ifndef REFLOW_PROFILES_H_
#define REFLOW_PROFILES_H_

// Number of temperature settings in a reflow profile
#define NUMPROFILETEMPS (48)

#define YAXIS (57)
#define XAXIS (12)

typedef struct {
	const char* name;
	const uint16_t temperatures[NUMPROFILETEMPS];
} profile;

typedef struct {
	const char* name;
	uint16_t temperatures[NUMPROFILETEMPS];
} ramprofile;

void Reflow_LoadCustomProfiles(void);
void Reflow_ValidateNV(void);
void Reflow_PlotProfile(int highlight);

int Reflow_GetProfileIdx(void);
int Reflow_SelectProfileIdx(int idx);
int Reflow_SelectEEProfileIdx(int idx);

int Reflow_GetEEProfileIdx(void);
int Reflow_SaveEEProfile(void);

/*
 * Queue a CUSTOM #1 update. Returns 0 when the points are legal, the oven
 * is in standby, and no commit is already running. Returns -1 otherwise.
 * Nothing is written to EEPROM or to the live curve here.
 *
 * Reflow_ServiceCustom1() performs that commit, one EEPROM page per call,
 * so the caller can return to the scheduler between pages. Call it until
 * it finishes. Returns 1 while work remains, 0 when the profile is stored
 * and selected, -1 when the write was refused and the live curve is
 * unchanged, -2 on an EEPROM error, and -3 when nothing is queued.
 * The live curve is updated only after every page has landed and the oven
 * is still in standby.
 */
int Reflow_WriteCustom1(const uint16_t* temps);
int Reflow_ServiceCustom1(void);
int Reflow_SelectCustom1(void);
void Reflow_ListProfiles(void);
const char* Reflow_GetProfileName(void);
uint16_t Reflow_GetSetpointAtIdx(uint8_t idx);
void Reflow_SetSetpointAtIdx(uint8_t idx, uint16_t value);
void Reflow_DumpProfile(int profile);

#endif /* REFLOW_PROFILES_H */
