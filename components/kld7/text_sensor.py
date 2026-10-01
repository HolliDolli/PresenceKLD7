import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import CONF_ID, CONF_TYPE, STATE_CLASS_MEASUREMENT, CONF_STATE_CLASS, DEVICE_CLASS_MOTION

from . import CONF_KLD7_ID, kld7_ns, Kld7

AUTO_LOAD = ["kld7"]
CONF_TYPE_RAW_JSON = "raw_json"
CONF_TYPE_JSON = "json"
CONF_TYPE_PDAT_JSON = "pdat_json"
# PDAT-Frames können mehr Targets liefern, als in einen einzelnen 255-Zeichen
# TextSensor-State passen (HA kappt sonst auf "unknown"). Targets 6-10 und
# 11-15 gehen in zwei weitere, optionale Sensoren.
CONF_TYPE_PDAT_JSON_2 = "pdat_json_2"
CONF_TYPE_PDAT_JSON_3 = "pdat_json_3"

Kld7TextSensor = kld7_ns.class_("TextSensor", text_sensor.TextSensor, cg.Component)

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(Kld7TextSensor),
        cv.GenerateID(CONF_KLD7_ID): cv.use_id(Kld7),
        cv.Optional(CONF_TYPE_RAW_JSON): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_TYPE_JSON): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_TYPE_PDAT_JSON): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_TYPE_PDAT_JSON_2): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_TYPE_PDAT_JSON_3): text_sensor.text_sensor_schema(
        ),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    kld7 = await cg.get_variable(config[CONF_KLD7_ID])

    if sensor_config := config.get(CONF_TYPE_RAW_JSON):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(kld7.register_raw_json_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_JSON):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(kld7.register_json_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_PDAT_JSON):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(kld7.register_pdat_json_sensor(sens))
    if sensor_config := config.get(CONF_TYPE_PDAT_JSON_2):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(kld7.register_pdat_json_sensor_2(sens))
    if sensor_config := config.get(CONF_TYPE_PDAT_JSON_3):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(kld7.register_pdat_json_sensor_3(sens))