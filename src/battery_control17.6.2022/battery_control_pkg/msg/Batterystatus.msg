# Power supply status constants
#battery state (0: Discharging ,1 :Charging ,2: Idle ,3 : Fault)

uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 0
uint8 POWER_SUPPLY_STATUS_IDLE = 2
uint8 POWER_SUPPLY_STATUS_ERORR = 3

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

std_msgs/Header header
float32 totalvoltage    #Total battery voltage
float32 instant_voltage # battery instant voltage
float32 celltempeture1 # Temperature in Degrees Celsius of the cell1
float32 celltempeture2 # Temperature in Degrees Celsius of the cell2
float32 Fettempeture1 # Temperature in Degrees Celsius of the FET
float32 Pcbtempeture1 # Temperature in Degrees Celsius of the PCB
float32 instant_current # Current in Amperes of the battery

float32 charge_current           # Current charge in Ah  (If unmeasured NaN)
float32 discharge_current        # Current discharge in Ah (If unmeasured NaN)
float32 battery_capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 Socpercentage       # Charge percentage Battery  State of Charge (SoC)  (%0-100)
uint8  battery_status     # The charging status as reported. Values defined above
#uint8   power_supply_health     # The battery health metric. Values defined above
uint8  battery_type        # The battery chemistry. Values defined above
bool   present_status          # True if the battery is usable

float32[] cells_volt_difference # The difference in voltage between each cell and the average cell voltage.
float32[] cells_voltage   # An array of individual cell voltages for each cell in the pack
float32   cells_avarege_volt # The average voltage of all cells in the pack
float32[] cells_temperature  # An array of individual cell temperatures for each cell in the pack
                            # If individual temperatures unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number