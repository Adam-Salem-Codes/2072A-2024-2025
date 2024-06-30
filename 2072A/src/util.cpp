#include "util.h"


/**
 * Checks if a device is plugged in at the given port.
 *
 * @param port The port number of the device.
 *
 * @return True if the device is plugged in and its type is not none or undefined, false otherwise.
 */
bool isPluggedIn(int port)
{
	return !(
		(pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none) ||
		(pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined));
}