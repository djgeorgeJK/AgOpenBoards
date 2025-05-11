

#define FW_MAJ          0
#define FW_MIN          1
#define FW_PATCH        3



/* EEPROM map */
#define EE_ADDR_READY        0     // EEread   2B
#define EE_ADDR_AOGCFG       6     // Machine module aogConfig
#define EE_ADDR_STEERSET    10    // Storage steerSettings 11B
#define EE_ADDR_SECTI_HEAD  22    // 2 B
#define EE_ADDR_SECTI       24    // 24 bytes
#define EE_ADDR_STEECFG     50    // Setup steerConfig;    13 B
#define EE_ADDR_NETWORK     64    // ConfigIP_t networkAddress;   //3 bytes


// if not in eeprom, overwrite
#define EEP_Ident 0xAA55



