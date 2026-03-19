

#define FW_MAJ          0
#define FW_MIN          1
#define FW_PATCH        12



/* EEPROM map */
#define EE_ADDR_READY        0    // EEread   2B
#define EE_ADDR_STEERSET     8    // Storage steerSettings 11B
#define EE_ADDR_STEECFG     20    // Setup steerConfig;    13 B
#define EE_ADDR_NETWORK     36    // ConfigIP_t networkAddress;   //3 bytes


#define EE_ADDR_SECTI_HEAD  48    // 2 B
#define EE_ADDR_SECTI       52    // 24 bytes
#define EE_ADDR_AOGCFG      76     // Machine module aogConfig 8B

// if not in eeprom, overwrite
#define EEP_Ident (uint16_t)(0xAA55)




