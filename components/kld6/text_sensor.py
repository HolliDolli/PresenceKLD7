# text_sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import KLD6Component

DEPENDENCIES = ["kld6"]

CONFIG_SCHEMA = text_sensor.text_sensor_schema()

async def to_code(config):
    var = await cg.get_variable(config[cv.GenerateID()])  # keine doppelte ID
    sens = await text_sensor.new_text_sensor(config)
    cg.add(var.set_direction_sensor(sens))
