

#define FW_MAJ          0
#define FW_MIN          1
#define FW_PATCH        14



/* EEPROM map */

/* Steer settings */
#define EE_ADDR_HEADER       0    // 2B
#define EE_ADDR_STEERSET     8    // Storage steerSettings 11B

/* Steer Config */
#define EE_ADDR_STEECFG     20    // SteerConfig_t steerConfig;    13 B
#define EE_ADDR_NETWORK     36    // ConfigIP_t networkAddress;   //3 bytes


/* Machine module EEPROM map */
#define EE_ADDR_SECTION_HEAD    48    // 2 B
#define EE_ADDR_SECTI           52    // 24 bytes
#define EE_ADDR_AOGCFG          76    // Machine module aogConfig 8B

// if not in eeprom, overwrite
#define EE_HEADER_DATA      (uint16_t)(0xAA55)




