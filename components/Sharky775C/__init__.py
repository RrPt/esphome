import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import maybe_simple_id
from esphome.components import sensor, text_sensor, uart
from esphome.const import (
    CONF_ID,
	CONF_ENERGY,
	CONF_FLOW,
	CONF_POWER,
    CONF_TEMPERATURE,
    CONF_TRIGGER_ID,
	DEVICE_CLASS_ENERGY,
	DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
	DEVICE_CLASS_VOLUME_FLOW_RATE,
	DEVICE_CLASS_VOLUME,
    DEVICE_CLASS_TIMESTAMP,
    DEVICE_CLASS_DURATION,
    ICON_MOLECULE_CO2,
    STATE_CLASS_MEASUREMENT,
    UNIT_WATT,
    UNIT_CELSIUS,
    UNIT_KILOWATT_HOURS,
)

CONF_VOLUME = "volume"
CONF_TEMPERATURE_IN = "temperature_in"
CONF_TEMPERATURE_OUT = "temperature_out"
CONF_ENERGY_CALC = "energy_calc"
CONF_ONTIME = "ontime"
CONF_COUNTER_TIME = "counter_time"
CONF_MEASURE_TIME = "measure_time"
CONF_NOOFAWAKEBYTES = "noOfAwakeBytes"
CONF_TIMEOUT ="timeout"
CONF_MBUSADR = "MBus_Adr"
CONF_SET_TIME = "set_time"

AUTO_LOAD = [ "sensor", "text_sensor" ]
DEPENDENCIES = ["uart"]

sharky775_ns = cg.esphome_ns.namespace("sharky775ns")
Sharky775Component = sharky775_ns.class_("Sharky775Component", cg.PollingComponent, uart.UARTDevice)

SetTimeAction = sharky775_ns.class_( "SetTimeAction", automation.Action)

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Sharky775Component),
            cv.Optional(CONF_TEMPERATURE_IN): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_OUT): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
			cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
			cv.Optional(CONF_ENERGY_CALC): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
			cv.Optional(CONF_POWER):sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ).extend({cv.Optional("max"):cv.float_range(1,60000)}),
			cv.Optional(CONF_FLOW): sensor.sensor_schema(
                unit_of_measurement="m3/s",
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLUME,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
			cv.Optional(CONF_VOLUME): sensor.sensor_schema(
                unit_of_measurement="m3",
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,   
                state_class=STATE_CLASS_MEASUREMENT,
            ),	
			cv.Optional(CONF_ONTIME): sensor.sensor_schema(
                unit_of_measurement="h",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_DURATION,   
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COUNTER_TIME): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MEASURE_TIME): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_NOOFAWAKEBYTES,default=480): cv.int_range(0,1000),
            cv.Optional(CONF_TIMEOUT,default=5000): cv.int_range(100,60000),
            cv.Optional(CONF_MBUSADR, default = 0 ): cv.float_range(0,255),
            cv.Optional(CONF_SET_TIME): cv.boolean,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_TEMPERATURE_IN in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_IN])
        cg.add(var.set_temperature_in_sensor(sens))

    if CONF_TEMPERATURE_OUT in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_OUT])
        cg.add(var.set_temperature_out_sensor(sens))

    if CONF_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY])
        cg.add(var.set_energy_sensor(sens))

    if CONF_ENERGY_CALC in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY_CALC])
        cg.add(var.set_energy_calc_sensor(sens))

    if CONF_POWER in config:
        sens = await sensor.new_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))
		
    if CONF_FLOW in config:
        sens = await sensor.new_sensor(config[CONF_FLOW])
        cg.add(var.set_flow_sensor(sens))
		
    if CONF_VOLUME in config:
        sens = await sensor.new_sensor(config[CONF_VOLUME])
        cg.add(var.set_volume_sensor(sens))

    if CONF_ONTIME in config:
        sens = await sensor.new_sensor(config[CONF_ONTIME])
        cg.add(var.set_ontime_sensor(sens))

    if CONF_NOOFAWAKEBYTES in config:
        cg.add(var.set_noOfAwakeBytes(config[CONF_NOOFAWAKEBYTES]))

    if CONF_TIMEOUT in config:
        cg.add(var.set_timeout(config[CONF_TIMEOUT]))

    if CONF_MBUSADR in config:
        cg.add(var.set_mbusAdr(config[CONF_MBUSADR]))

    if CONF_COUNTER_TIME in config:
        textX = await text_sensor.new_text_sensor(config[CONF_COUNTER_TIME])
        cg.add(var.set_counter_time(textX))

    if CONF_MEASURE_TIME in config:
        textX = await text_sensor.new_text_sensor(config[CONF_MEASURE_TIME])
        cg.add(var.set_measure_time(textX))

    if "max" in config:
        cg.add(var.set_maxPower(config["max"]))

SET_TIME_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(Sharky775Component),
    }
)

@automation.register_action(
    "sharky775ns.set_time", SetTimeAction, SET_TIME_ACTION_SCHEMA
)
async def sharky775ns_set_time_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)