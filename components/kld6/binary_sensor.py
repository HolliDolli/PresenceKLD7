# components/kld6/binary_sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import KLD6Component

DEPENDENCIES = ["kld6"]

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(KLD6Component),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = await cg.get_variable(config[cv.GenerateID()])
    sens = await binary_sensor.new_binary_sensor(config)
    cg.add(var.set_presence_sensor(sens))
