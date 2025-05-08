

#define SERIAL_BUFFER_SIZE		512


void EthernetStart(void);
void Process_RTK_FromUDP(void);
void EthernetTask(void);

typedef struct ConfigIP
{
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
}ConfigIP_t;
