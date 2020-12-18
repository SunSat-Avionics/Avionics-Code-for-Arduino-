/* all devices on the bus are happy with 10MHz clock */
#define CLOCK_RATE 10000000

uint32_t readSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t numBytes);
void writeSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t data);
