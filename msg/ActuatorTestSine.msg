# Custom sine wave test output for additional servo channel
# This does not affect normal control allocation outputs

uint64 timestamp				# time since system start (microseconds)

uint8 MAX_NUM_CHANNELS = 8		# Maximum number of test channels

bool enabled					# Enable sine wave test output
uint8 channel					# Target channel number (0-7)
float32 frequency				# Sine wave frequency in Hz
float32 amplitude				# Sine wave amplitude (0.0 to 1.0)
float32 offset					# DC offset (-1.0 to 1.0)
float32[8] output				# Output values for each channel, NaN if not active
