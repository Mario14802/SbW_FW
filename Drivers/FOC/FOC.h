typedef struct {
    float alpha;
    float beta;
} Float2D;

typedef struct {
    float vbus_voltage;   // Voltage on the DC bus
    Float2D Ialpha_beta;  // Clarke-transformed currents
    uint32_t timestamp;   // Input timestamp
} MeasurementData;



MeasurementData AlphaBetaFrameController_on_measurement(
    float *vbus_voltage, float *currents, uint32_t input_timestamp);
