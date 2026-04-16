#include "ati_sensor/ft_sensor.h"
#include <cmath>
class ATISensorDriver {
private:
std::unique_ptr<FTSensor> sensor_;
CalibrationMatrix calibration_;
std::array<double, 6> bias_;
public:
bool initialize(const std::string& ip, int port) {
sensor_ = std::make_unique<FTSensor>();
if (!sensor_->connect(ip, port)) {
return false;
}
// Load calibration matrix from device
22
calibration_ = sensor_->getCalibration();
return true;
}
Wrench readWrench() {
auto voltages = sensor_->readRaw();
Wrench wrench;
// Apply calibration: w = C * v
for (int i = 0; i < 6; i++) {
wrench[i] = bias_[i];
for (int j = 0; j < 6; j++) {
wrench[i] += calibration_(i,j) * voltages[j];
}
}
return wrench;
}
void tare() {
// Zero the sensor
auto current = readWrench();
for (int i = 0; i < 6; i++) {
bias_[i] -= current[i];
}
}
};
