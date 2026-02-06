import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor
from esphome.const import CONF_ID, CONF_TYPE, STATE_CLASS_MEASUREMENT, CONF_STATE_CLASS, DEVICE_CLASS_MOTION

from . import CONF_KLD7_ID, kld7_ns, Kld7

AUTO_LOAD = ["kld7"]
CONF_TYPE_RAW_DETECTION = "raw_detection"
CONF_TYPE_DETECTION = "detection"
CONF_TYPE_RAW_DIRECTION_AWAY_FROM_SENSOR = "raw_direction_away_from_sensor"
CONF_TYPE_FILTERED_DETECTION = "filtered_detection"

CONF_OPTION_MIN_DISTANCE = "min_distance"
CONF_OPTION_MAX_DISTANCE = "max_distance"
CONF_OPTION_MIN_ANGLE = "min_angle"
CONF_OPTION_MAX_ANGLE = "max_angle"
CONF_OPTION_MIN_POINTS = "min_points"
CONF_OPTION_TIMEOUT = "timeout"

Kld7BinarySensor = kld7_ns.class_("BinarySensor", binary_sensor.BinarySensor, cg.Component)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(Kld7BinarySensor),
        cv.GenerateID(CONF_KLD7_ID): cv.use_id(Kld7),
        cv.Optional(CONF_TYPE_RAW_DETECTION): binary_sensor.binary_sensor_schema(
            device_class= DEVICE_CLASS_MOTION
        ),
        cv.Optional(CONF_TYPE_DETECTION): binary_sensor.binary_sensor_schema(
            device_class= DEVICE_CLASS_MOTION
        ),
        cv.Optional(CONF_TYPE_RAW_DIRECTION_AWAY_FROM_SENSOR): binary_sensor.binary_sensor_schema(
        ),
        cv.Optional(CONF_TYPE_FILTERED_DETECTION): binary_sensor.binary_sensor_schema(
            device_class= DEVICE_CLASS_MOTION
        ).extend({
            cv.Optional(CONF_OPTION_MIN_DISTANCE, default=0): cv.int_range(min=0, max=3000),
            cv.Optional(CONF_OPTION_MAX_DISTANCE, default=3000): cv.int_range(min=0, max=3000),
            cv.Optional(CONF_OPTION_MIN_ANGLE, default=-90): cv.float_range(min=-90, max=90),
            cv.Optional(CONF_OPTION_MAX_ANGLE, default=-90): cv.float_range(min=-90, max=90),
            cv.Optional(CONF_OPTION_MIN_POINTS, default=5): cv.int_range(min=1, max=1024),
            cv.Optional(CONF_OPTION_TIMEOUT, default=1000): cv.int_range(min=1, max=60000)
        })
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    kld7 = await cg.get_variable(config[CONF_KLD7_ID])

    if sensor_config := config.get(CONF_TYPE_RAW_DETECTION):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(kld7.register_raw_detection_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_DETECTION):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(kld7.register_detection_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_RAW_DIRECTION_AWAY_FROM_SENSOR):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(kld7.register_raw_direction_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_FILTERED_DETECTION):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(kld7.register_filtered_detection_sensor(sens))
        cg.add(kld7.set_filtered_detection_sensor_min_distance(sensor_config[CONF_OPTION_MIN_DISTANCE]))
        cg.add(kld7.set_filtered_detection_sensor_max_distance(sensor_config[CONF_OPTION_MAX_DISTANCE]))
        cg.add(kld7.set_filtered_detection_sensor_min_angle(sensor_config[CONF_OPTION_MIN_ANGLE]))
        cg.add(kld7.set_filtered_detection_sensor_max_angle(sensor_config[CONF_OPTION_MAX_ANGLE]))
        cg.add(kld7.set_filtered_detection_sensor_min_points(sensor_config[CONF_OPTION_MIN_POINTS]))
        cg.add(kld7.set_filtered_detection_sensor_timeout(sensor_config[CONF_OPTION_TIMEOUT]))