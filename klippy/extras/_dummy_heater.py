# Dummy heater implementation used when a heater pin should be ignored.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import threading


KELVIN_TO_CELSIUS = -273.15


class _DummyControl:
    """Minimal control shim so the heater interface remains consistent."""

    def __init__(self) -> None:
        self._profile = {"name": "dummy", "control": "none"}

    def get_profile(self):
        return self._profile

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return False


class DummyHeater:
    """Heater stub that skips PWM allocation while keeping sensor feedback."""

    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.config = config
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]
        self.sensor = sensor
        # Consume heater-specific config fields so they aren't flagged as unused
        self.heater_pin = config.get("heater_pin", None)
        self.lock = threading.Lock()
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        self.smooth_time = config.getfloat("smooth_time", 1.0, above=0.0)
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.min_extrude_temp = config.getfloat(
            "min_extrude_temp",
            170.0,
            minval=self.min_temp,
            maxval=self.max_temp,
        )
        is_fileoutput = (
            self.printer.get_start_args().get("debugoutput") is not None
        )
        self.can_extrude = self.min_extrude_temp <= 0.0 or is_fileoutput
        self.target_temp = 0.0
        self.last_temp = 0.0
        self.smoothed_temp = 0.0
        self.last_temp_time = 0.0
        self.last_pwm_value = 0.0
        self.control = _DummyControl()
        config.get("control", None)
        config.getfloat("pid_kp", None)
        config.getfloat("pid_ki", None)
        config.getfloat("pid_kd", None)
        config.getfloat("max_delta", None)
        self.max_power = config.getfloat("max_power", 1.0, above=0.0, maxval=1.0)
        if self.sensor is not None:
            self.sensor.setup_minmax(self.min_temp, self.max_temp)
            self.sensor.setup_callback(self._sensor_callback)
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE",
            "HEATER",
            self.short_name,
            self.cmd_SET_HEATER_TEMPERATURE,
            desc="Sets a dummy heater temperature target",
        )
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

    def get_name(self):
        return self.name

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def get_pwm_delay(self):
        return self.smooth_time

    def set_inv_smooth_time(self, inv_smooth_time):
        self.inv_smooth_time = inv_smooth_time

    def set_control(self, control, keep_target=True):
        with self.lock:
            old_control = self.control
            self.control = control
            if not keep_target:
                self.target_temp = 0.0
        return old_control

    def get_control(self):
        return self.control

    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        with self.lock:
            self.target_temp = target_temp

    def set_pwm(self, read_time, value):
        # Dummy heaters never drive PWM hardware but we still track the request.
        with self.lock:
            self.last_pwm_value = value

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp)
            )
        with self.lock:
            self.target_temp = degrees

    def get_temp(self, eventtime):
        with self.lock:
            return self.smoothed_temp, self.target_temp

    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp
            )

    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.0
        return is_active, "%s: target=%.0f temp=%.1f pwm=%.3f (dummy)" % (
            self.short_name,
            target_temp,
            last_temp,
            last_pwm_value,
        )

    def get_status(self, eventtime):
        with self.lock:
            smoothed_temp = self.smoothed_temp
            target_temp = self.target_temp
            last_pwm_value = self.last_pwm_value
        return {
            "temperature": round(smoothed_temp, 2),
            "target": target_temp,
            "power": last_pwm_value,
            "pid_profile": self.control.get_profile()["name"],
        }

    def is_adc_faulty(self):
        with self.lock:
            return self.last_temp > self.max_temp or self.last_temp < self.min_temp

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float("TARGET", 0.0)
        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(self, temp)

    def _sensor_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp_time = read_time
            self.last_temp = temp
            if time_diff <= 0.0:
                self.smoothed_temp = temp
                return
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += (temp - self.smoothed_temp) * adj_time
            if self.min_extrude_temp <= 0.0:
                self.can_extrude = True
            else:
                self.can_extrude = self.smoothed_temp >= self.min_extrude_temp

    def _handle_shutdown(self):
        with self.lock:
            self.target_temp = 0.0
            self.last_pwm_value = 0.0