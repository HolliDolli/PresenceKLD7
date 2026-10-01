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

CONF_OPTION_MIN_POINTS = "min_points"
CONF_OPTION_TIMEOUT = "timeout"

# Neue Filterkriterien für filtered_detection: Speed-Persistenz + PDAT-Kohärenz
# statt Winkel/Distanz-Fenster. Begründung: siehe kld7.h / Analyse von
# KLD7-log-Kind-Erw.csv ("direction changed" und "magnitude" sind bei einem
# orthogonal zum Sensor verlaufenden Gehweg keine brauchbaren Kriterien).
CONF_OPTION_SPEED_MIN = "speed_min"
CONF_OPTION_MIN_RUN_FRAMES = "min_run_frames"
CONF_OPTION_MAX_FRAME_GAP = "max_frame_gap"
CONF_OPTION_PDAT_DISTANCE_TOLERANCE = "pdat_distance_tolerance"
CONF_OPTION_PDAT_SPEED_MIN = "pdat_speed_min"
CONF_OPTION_VALIDATION_WINDOW = "validation_window"

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
            cv.Optional(CONF_OPTION_MIN_POINTS, default=5): cv.int_range(min=1, max=1024),
            cv.Optional(CONF_OPTION_TIMEOUT, default=1000): cv.int_range(min=1, max=60000),
            # Speed-Gate: |speed| muss diesen Wert erreichen, um als Bewegung
            # zu zählen (Einheit wie vom KLD7 geliefert, default 0.8).
            cv.Optional(CONF_OPTION_SPEED_MIN, default=0.8): cv.float_range(min=0.0, max=50.0),
            # Persistenz-Gate: Mindestanzahl aufeinanderfolgender Frames über
            # speed_min, bevor eine Detection überhaupt in Frage kommt.
            cv.Optional(CONF_OPTION_MIN_RUN_FRAMES, default=4): cv.int_range(min=1, max=255),
            # Maximale Lücke (ms) zwischen zwei Frames, die noch als
            # "derselbe Run" gilt.
            cv.Optional(CONF_OPTION_MAX_FRAME_GAP, default=500): cv.int_range(min=1, max=60000),
            # PDAT-Kohärenz-Gate: Toleranz (cm) zwischen TDAT-Distanz und
            # einem validierenden PDAT-Punkt.
            cv.Optional(CONF_OPTION_PDAT_DISTANCE_TOLERANCE, default=80): cv.int_range(min=0, max=3000),
            # Mindestgeschwindigkeit eines PDAT-Punktes, damit er als
            # Validierung zählt.
            cv.Optional(CONF_OPTION_PDAT_SPEED_MIN, default=0.8): cv.float_range(min=0.0, max=50.0),
            # Zeitfenster (ms) ab Run-Start, in dem eine PDAT-Validierung
            # noch akzeptiert wird.
            cv.Optional(CONF_OPTION_VALIDATION_WINDOW, default=1200): cv.int_range(min=1, max=60000),
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
        cg.add(kld7.set_filtered_detection_sensor_min_points(sensor_config[CONF_OPTION_MIN_POINTS]))
        cg.add(kld7.set_filtered_detection_sensor_timeout(sensor_config[CONF_OPTION_TIMEOUT]))
        cg.add(kld7.set_filtered_detection_speed_min(sensor_config[CONF_OPTION_SPEED_MIN]))
        cg.add(kld7.set_filtered_detection_min_run_frames(sensor_config[CONF_OPTION_MIN_RUN_FRAMES]))
        cg.add(kld7.set_filtered_detection_max_frame_gap(sensor_config[CONF_OPTION_MAX_FRAME_GAP]))
        cg.add(kld7.set_filtered_detection_pdat_distance_tolerance(sensor_config[CONF_OPTION_PDAT_DISTANCE_TOLERANCE]))
        cg.add(kld7.set_filtered_detection_pdat_speed_min(sensor_config[CONF_OPTION_PDAT_SPEED_MIN]))
        cg.add(kld7.set_filtered_detection_validation_window(sensor_config[CONF_OPTION_VALIDATION_WINDOW]))