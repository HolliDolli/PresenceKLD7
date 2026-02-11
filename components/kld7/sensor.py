import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (CONF_ID, CONF_TYPE, UNIT_KILOMETER_PER_HOUR, STATE_CLASS_MEASUREMENT, UNIT_DEGREES, UNIT_CENTIMETER,
	DEVICE_CLASS_DISTANCE, DEVICE_CLASS_SPEED, UNIT_DECIBEL, DEVICE_CLASS_IRRADIANCE, DEVICE_CLASS_WEIGHT)
from . import CONF_KLD7_ID, kld7_ns, Kld7

AUTO_LOAD = ["kld7"]
CONF_TYPE_SPEED = "speed"
CONF_TYPE_RAW_SPEED = "raw_speed"
CONF_TYPE_AVG_SPEED = "avg_speed"
CONF_TYPE_AVG_SIZE = "avg_size"
CONF_TYPE_POINTS = "points"
CONF_TYPE_RAW_ANGLE = "raw_angle"
CONF_TYPE_RAW_DISTANCE = "raw_distance"
CONF_TYPE_RAW_MAGNITUDE = "raw_magnitude"

Kld7Sensor = kld7_ns.class_("Sensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(Kld7Sensor),
        cv.GenerateID(CONF_KLD7_ID): cv.use_id(Kld7),
        #cv.Required(CONF_TYPE): cv.one_of(CONF_TYPE_SPEED, CONF_TYPE_RAW_SPEED, lower=True)
		cv.Optional(CONF_TYPE_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_SPEED
		),
        cv.Optional(CONF_TYPE_AVG_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_SPEED
		),
        cv.Optional(CONF_TYPE_AVG_SIZE): sensor.sensor_schema(
            unit_of_measurement="kg",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_WEIGHT
		),
        cv.Optional(CONF_TYPE_RAW_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_SPEED
		),
        cv.Optional(CONF_TYPE_RAW_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT
		),
        cv.Optional(CONF_TYPE_POINTS): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT
		),
        cv.Optional(CONF_TYPE_RAW_DISTANCE): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_CENTIMETER,
            device_class=DEVICE_CLASS_DISTANCE
		),
        cv.Optional(CONF_TYPE_RAW_MAGNITUDE): sensor.sensor_schema(
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement="W/m²",
            device_class=DEVICE_CLASS_IRRADIANCE
		)
    }
)


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
    )
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    kld7 = await cg.get_variable(config[CONF_KLD7_ID])
    if sensor_config := config.get(CONF_TYPE_SPEED):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_speed_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_RAW_SPEED):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_raw_speed_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_AVG_SPEED):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_avg_speed_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_AVG_SIZE):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_avg_size_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_POINTS):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_points_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_RAW_ANGLE):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_raw_angle_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_RAW_DISTANCE):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_raw_distance_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_RAW_MAGNITUDE):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(kld7.register_raw_magnitude_sensor(sens))