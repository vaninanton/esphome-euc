import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor, text_sensor
from esphome.const import CONF_ID

AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]

my_ns = cg.esphome_ns.namespace("veteran")
VeteranComponent = my_ns.class_("VeteranComponent", cg.Component)

BINARY_FIELDS = {
    "charging": "binary_sensor_charging",
    "low_power_mode": "binary_sensor_low_power_mode",
    "high_speed_mode": "binary_sensor_high_speed_mode",
}

SENSOR_FIELDS = {
    "auto_off": "sensor_auto_off",
    "battery_percentage": "sensor_battery_percentage",
    "bms_left_current": "sensor_bms_left_current",
    "bms_right_current": "sensor_bms_right_current",
    "charging_stop_voltage": "sensor_charging_stop_voltage",
    "power": "sensor_power",

    "bms_left_temp_1": "sensor_bms_left_temp_1",
    "bms_left_temp_2": "sensor_bms_left_temp_2",
    "bms_left_temp_3": "sensor_bms_left_temp_3",
    "bms_left_temp_4": "sensor_bms_left_temp_4",
    "bms_left_temp_5": "sensor_bms_left_temp_5",
    "bms_left_temp_6": "sensor_bms_left_temp_6",
    "bms_right_temp_1": "sensor_bms_right_temp_1",
    "bms_right_temp_2": "sensor_bms_right_temp_2",
    "bms_right_temp_3": "sensor_bms_right_temp_3",
    "bms_right_temp_4": "sensor_bms_right_temp_4",
    "bms_right_temp_5": "sensor_bms_right_temp_5",
    "bms_right_temp_6": "sensor_bms_right_temp_6",

    "temperature_motor": "sensor_temperature_motor",
    "temperature_controller": "sensor_temperature_controller",
    "tho_ra": "sensor_tho_ra",
    "mileage_current": "sensor_mileage_current",
    "mileage_total": "sensor_mileage_total",
    "voltage": "sensor_voltage",
}

TEXT_SENSOR_FIELDS = {
    "firmware_version": "text_sensor_firmware_version",
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(VeteranComponent),
    **{cv.Optional(k): binary_sensor.binary_sensor_schema().extend(cv.COMPONENT_SCHEMA) for k in BINARY_FIELDS},
    **{cv.Optional(k): sensor.sensor_schema().extend(cv.COMPONENT_SCHEMA) for k in SENSOR_FIELDS},
    **{cv.Optional(k): text_sensor.text_sensor_schema().extend(cv.COMPONENT_SCHEMA) for k in TEXT_SENSOR_FIELDS},
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for key, setter in BINARY_FIELDS.items():
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, setter)(sens))

    for key, setter in SENSOR_FIELDS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))

    for key, setter in TEXT_SENSOR_FIELDS.items():
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
    
