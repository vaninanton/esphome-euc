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
    "bms_current_left": "sensor_bms_current_left",
    "bms_current_right": "sensor_bms_current_right",
    "charging_stop_voltage": "sensor_charging_stop_voltage",

    "bms_left_cell_01": "sensor_bms_left_cell_01",
    "bms_left_cell_02": "sensor_bms_left_cell_02",
    "bms_left_cell_03": "sensor_bms_left_cell_03",
    "bms_left_cell_04": "sensor_bms_left_cell_04",
    "bms_left_cell_05": "sensor_bms_left_cell_05",
    "bms_left_cell_06": "sensor_bms_left_cell_06",
    "bms_left_cell_07": "sensor_bms_left_cell_07",
    "bms_left_cell_08": "sensor_bms_left_cell_08",
    "bms_left_cell_09": "sensor_bms_left_cell_09",
    "bms_left_cell_10": "sensor_bms_left_cell_10",
    "bms_left_cell_11": "sensor_bms_left_cell_11",
    "bms_left_cell_12": "sensor_bms_left_cell_12",
    "bms_left_cell_13": "sensor_bms_left_cell_13",
    "bms_left_cell_14": "sensor_bms_left_cell_14",
    "bms_left_cell_15": "sensor_bms_left_cell_15",
    "bms_left_cell_16": "sensor_bms_left_cell_16",
    "bms_left_cell_17": "sensor_bms_left_cell_17",
    "bms_left_cell_18": "sensor_bms_left_cell_18",
    "bms_left_cell_19": "sensor_bms_left_cell_19",
    "bms_left_cell_20": "sensor_bms_left_cell_20",
    "bms_left_cell_21": "sensor_bms_left_cell_21",
    "bms_left_cell_22": "sensor_bms_left_cell_22",
    "bms_left_cell_23": "sensor_bms_left_cell_23",
    "bms_left_cell_24": "sensor_bms_left_cell_24",
    "bms_left_cell_25": "sensor_bms_left_cell_25",
    "bms_left_cell_26": "sensor_bms_left_cell_26",
    "bms_left_cell_27": "sensor_bms_left_cell_27",
    "bms_left_cell_28": "sensor_bms_left_cell_28",
    "bms_left_cell_29": "sensor_bms_left_cell_29",
    "bms_left_cell_30": "sensor_bms_left_cell_30",
    "bms_left_cell_31": "sensor_bms_left_cell_31",
    "bms_left_cell_32": "sensor_bms_left_cell_32",
    "bms_left_cell_33": "sensor_bms_left_cell_33",
    "bms_left_cell_34": "sensor_bms_left_cell_34",
    "bms_left_cell_35": "sensor_bms_left_cell_35",
    "bms_left_cell_36": "sensor_bms_left_cell_36",
    
    "bms_right_cell_01": "sensor_bms_right_cell_01",
    "bms_right_cell_02": "sensor_bms_right_cell_02",
    "bms_right_cell_03": "sensor_bms_right_cell_03",
    "bms_right_cell_04": "sensor_bms_right_cell_04",
    "bms_right_cell_05": "sensor_bms_right_cell_05",
    "bms_right_cell_06": "sensor_bms_right_cell_06",
    "bms_right_cell_07": "sensor_bms_right_cell_07",
    "bms_right_cell_08": "sensor_bms_right_cell_08",
    "bms_right_cell_09": "sensor_bms_right_cell_09",
    "bms_right_cell_10": "sensor_bms_right_cell_10",
    "bms_right_cell_11": "sensor_bms_right_cell_11",
    "bms_right_cell_12": "sensor_bms_right_cell_12",
    "bms_right_cell_13": "sensor_bms_right_cell_13",
    "bms_right_cell_14": "sensor_bms_right_cell_14",
    "bms_right_cell_15": "sensor_bms_right_cell_15",
    "bms_right_cell_16": "sensor_bms_right_cell_16",
    "bms_right_cell_17": "sensor_bms_right_cell_17",
    "bms_right_cell_18": "sensor_bms_right_cell_18",
    "bms_right_cell_19": "sensor_bms_right_cell_19",
    "bms_right_cell_20": "sensor_bms_right_cell_20",
    "bms_right_cell_21": "sensor_bms_right_cell_21",
    "bms_right_cell_22": "sensor_bms_right_cell_22",
    "bms_right_cell_23": "sensor_bms_right_cell_23",
    "bms_right_cell_24": "sensor_bms_right_cell_24",
    "bms_right_cell_25": "sensor_bms_right_cell_25",
    "bms_right_cell_26": "sensor_bms_right_cell_26",
    "bms_right_cell_27": "sensor_bms_right_cell_27",
    "bms_right_cell_28": "sensor_bms_right_cell_28",
    "bms_right_cell_29": "sensor_bms_right_cell_29",
    "bms_right_cell_30": "sensor_bms_right_cell_30",
    "bms_right_cell_31": "sensor_bms_right_cell_31",
    "bms_right_cell_32": "sensor_bms_right_cell_32",
    "bms_right_cell_33": "sensor_bms_right_cell_33",
    "bms_right_cell_34": "sensor_bms_right_cell_34",
    "bms_right_cell_35": "sensor_bms_right_cell_35",
    "bms_right_cell_36": "sensor_bms_right_cell_36",

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
    
