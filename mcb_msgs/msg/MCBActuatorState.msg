Header header
int32 device_id  # Position in EtherCAT chain

int32 encoder_count  # The number of ticks as reported by the encoder
float64 position  # The position of the motor (in radians)
float64 encoder_velocity  # The velocity measured in encoder ticks per second
float64 velocity  #The velocity in radians per second

bool calibration_reading  # the value of the last calibration reading: low (false) or high (true)
bool calibration_rising_edge_valid  # Is the last_callibration_rising_edge_ field valid?
bool calibration_falling_edge_valid  # Is the last_callibration_falling_edge_ field valid?
float64 last_calibration_rising_edge  # The position of the motor the last time the calibration switch went from low to high.
float64 last_calibration_falling_edge  # The position of the motor the last time the calibration switch went from high to low.

bool index_reading    # The value of the encoder index pin.  True when index pin is asserted
bool index_valid      # True when index_position field is valid
float64 index_position  # The position of the motor (in radians) the last time the encoder index was asserted

bool is_enabled  # Enable status
bool halted  # indicates if the motor is halted. A motor can be halted because of voltage or communication problems

float64 pwm_ratio       # Ratio -1.0 to 1.0 used for pwm output
float64 supply_voltage  # Motor controller board supply voltage (in Volts)

float64 last_commanded_current  # The current computed based on the effort specified in the ActuatorCommand (in amps)
float64 last_executed_current  # The actual current requested after safety limits were enforced (in amps)
float64 last_measured_current  # The measured current (in amps)

float64 last_commanded_effort  # The torque requested in the previous ActuatorCommand (in Nm)
float64 last_executed_effort  # The torque applied after safety limits were enforced (in Nm)
float64 last_measured_effort  # The measured torque (in Nm)

float64 motor_voltage  # Motor voltage (in volts)

int32 num_encoder_errors  # The number of invalid encoder signal transitions

float64 zero_offset  # A bias applied to the position value when reported.  This value is written once after calibration. The reported position is the hardware's actual position minus the zero offset
