
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



/*  USER NUMBER 1  aogConfig.user1 */
#define UN1_WS_DIS_SEC        0x01                  // when 1 turning off Work switch, will not off sections
