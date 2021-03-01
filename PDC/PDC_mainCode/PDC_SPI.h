void readSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t numBytes, uint8_t *result);
void readSPIwithDummy(uint8_t deviceSelect, uint8_t registerSelect, uint8_t numBytes, uint8_t *result);
void writeSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t data);
