import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
#, sensor, binary_sensor, text_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

kld6_ns = cg.esphome_ns.namespace("kld6")
KLD6Component = kld6_ns.class_("KLD6Component", cg.Component, uart.UARTDevice)

CONF_UART_ID = "uart_id"
CONF_DISTANCE = "distance"
CONF_SPEED = "speed"
CONF_PRESENCE_TIMEOUT = "presence_timeout"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(KLD6Component),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_PRESENCE_TIMEOUT, default="3s"): cv.positive_time_period,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    # Component zuerst registrieren
    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[cv.GenerateID()], uart_comp)
    await cg.register_component(var, config)

    cg.add(var.set_presence_timeout(config[CONF_PRESENCE_TIMEOUT].total_milliseconds))
