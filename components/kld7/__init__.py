import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@fabianonline"]

DEPENDENCIES = ["uart"]

kld7_ns = cg.esphome_ns.namespace("kld7")
Kld7 = kld7_ns.class_("Kld7", cg.Component, uart.UARTDevice)
MULTI_CONF = True

CONF_KLD7_ID = "kld7_id"
CONF_KLD7_INVERT_ANGLE = "invert_angle"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Kld7),
        cv.Optional(CONF_KLD7_INVERT_ANGLE, default = False): cv.boolean
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    component = await cg.register_component(var, config)
    cg.add(var.set_invert_angle(config[CONF_KLD7_INVERT_ANGLE]))
    await uart.register_uart_device(var, config)
