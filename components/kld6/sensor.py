# components/kld6/sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import KLD6Component

DEPENDENCIES = ["kld6"]

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(KLD6Component),
    cv.Optional("distance"): sensor.sensor_schema(),
    cv.Optional("speed"): sensor.sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = await cg.get_variable(config[cv.GenerateID()])
    if "distance" in config:
        sens = await sensor.new_sensor(config["distance"])
        cg.add(var.set_distance_sensor(sens))
    if "speed" in config:
        sens = await sensor.new_sensor(config["speed"])
        cg.add(var.set_speed_sensor(sens))
