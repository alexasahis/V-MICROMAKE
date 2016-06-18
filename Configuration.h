//固件修改更新QQ群：224981313
//固件配套的3D打印机学习套件购买地址：http://item.taobao.com/item.htm?id=42916612391
//网盘资料：http://yunpan.taobao.com/s/19pI3jQStOL
//视频教程 - v1,v2版机型软件设置，自动调平
//优酷地址：http://www.youku.com/playlist_show/id_23218776.html
//视频教程 - v3版机型组装部分，即初八以后发货的组装视频
//优酷地址：http://www.youku.com/playlist_show/id_23522533.html

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//===========================================================================
//==========================MICROMAKE 3D打印机配套固件 ======================
//===========================================================================
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ 
#define STRING_CONFIG_H_AUTHOR "(MICROMAKE KOSSEL)" 

#define SERIAL_PORT 0

// This determines the communication speed of the printer 
#define BAUDRATE 115200

//// Motherboards
// 10 = Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
// 11 = Gen7 v1.1, v1.2 = 11
// 12 = Gen7 v1.3
// 13 = Gen7 v1.4
// 2  = Cheaptronic v1.0
// 20 = Sethi 3D_1
// 3  = MEGA/RAMPS up to 1.2 = 3
// 33 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
// 34 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
// 35 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Fan)
// 4  = Duemilanove w/ ATMega328P pin assignment
// 5  = Gen6
// 51 = Gen6 deluxe
// 6  = Sanguinololu < 1.2
// 62 = Sanguinololu 1.2 and above
// 63 = Melzi
// 64 = STB V1.1
// 65 = Azteeg X1
// 66 = Melzi with ATmega1284 (MaKr3d version)
// 67 = Azteeg X3
// 68 = Azteeg X3 Pro
// 7  = Ultimaker
// 71 = Ultimaker (Older electronics. Pre 1.5.4. This is rare)
// 72 = Ultimainboard 2.x (Uses TEMP_SENSOR 20)
// 77 = 3Drag Controller
// 8  = Teensylu
// 80 = Rumba
// 81 = Printrboard (AT90USB1286)
// 82 = Brainwave (AT90USB646)
// 83 = SAV Mk-I (AT90USB1286)
// 9  = Gen3+
// 70 = Megatronics
// 701= Megatronics v2.0
// 702= Minitronics v1.0
// 90 = Alpha OMCA board
// 91 = Final OMCA board
// 301= Rambo
// 21 = Elefu Ra Board (v3)

#ifndef MOTHERBOARD
#define MOTHERBOARD 33 // RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
#endif

// Define this to set a custom name for your generic Mendel,
#define CUSTOM_MENDEL_NAME "UM v2.4"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

//This defines the number of extruders
#define EXTRUDERS 1

//// 电源电压电流
//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)

#define POWER_SUPPLY 1

//===========================================================================
//====================MICROMAKE 3D打印机 三角洲结构配置======================
//===========================================================================
//
//
//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
#define DELTA

// //减小这个数值，来缓解卡顿现象，如修改为120进行测试。
// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).

#define DELTA_SEGMENTS_PER_SECOND 160

// 碳杆长度，从一端球中心到另一端球中心的距离 大小调整此参数
//
// Center-to-center distance of the holes in the diagonal push rods.
//#define DELTA_DIAGONAL_ROD 214// mm
#define DELTA_DIAGONAL_ROD 212.4// mm was 214

// 打印头到滑杆水平距离 凹凸调整此参数
// Horizontal offset from middle of printer to smooth rod center.
//#define DELTA_SMOOTH_ROD_OFFSET 160// mm
#define DELTA_SMOOTH_ROD_OFFSET 155.80// mm

// 效应器球中心和打印头的水平距离
//#define DELTA_EFFECTOR_OFFSET 24.0 // mm
// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 23.0 // mm was 24

// 滑车球中心到滑杆水平距离
// Horizontal offset of the universal joints on the carriages.
//#define DELTA_CARRIAGE_OFFSET 22.0 // mm
#define DELTA_CARRIAGE_OFFSET 21.0 // mm was 22.0

// 三角洲半径.（打印头到滑杆水平距离-效应器球中心和打印头的水平距离-滑车球中心到滑杆水平距离）
// Horizontal distance bridged by diagonal push rods when effector is centered.
#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

// 打印半径
// #define DELTA_PRINTABLE_RADIUS 90.0
// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#define DELTA_PRINTABLE_RADIUS 80.0


#define SIN_60 0.8660254037844386
#define COS_60 0.5
#define DELTA_TOWER1_X -SIN_60*DELTA_RADIUS
#define DELTA_TOWER1_Y -COS_60*DELTA_RADIUS
#define DELTA_TOWER2_X SIN_60*DELTA_RADIUS
#define DELTA_TOWER2_Y -COS_60*DELTA_RADIUS
#define DELTA_TOWER3_X 0.0
#define DELTA_TOWER3_Y DELTA_RADIUS

#define DELTA_DIAGONAL_ROD_2 pow(DELTA_DIAGONAL_ROD,2)

//===========================================================================
//========================MICROMAKE 3D打印机 传感器设置======================
//===========================================================================

//配置传感器，根据挤出机个数来配置连接传感器数量。如果只有1个挤出机，则只需要开启传感器0接口和热床接口即可。
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is Mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 11 is 100k beta 3950 1% thermistor (4.7k pullup)
// 12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
// 13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE" 
// 20 is the PT100 circuit found in the Ultimainboard V2.x
// 60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
//
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
// 147 is Pt100 with 4k7 pullup
// 110 is Pt100 with 1k pullup (non standard)

#define TEMP_SENSOR_0 5 //设置传感器0接口连接的传感器类型编号，类型根据上面说明设置相应的编号 // translation: // Set the sensor type number 0 interface sensor connection, type instructions to set up the corresponding number according to the above)
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0  //设置传感器2接口连接的传感器编号，0表示关闭该端口 // translation: Sensor 2 sensor number setting interface, and 0 means shut down the port

// vera: what the heck is that???
// should be 8 or 12, perhaps? 
// #define TEMP_SENSOR_BED 0 //设置热床传感器端口连接的传感器类型。该项如果设置错误将影响加热床温度控制 // translation: Setting sensor type hot bed sensor port. If the error affects the temperature control of the heating bed
#define TEMP_SENSOR_BED 5
// translation: Just add hot bed support #define TEMP_SENSOR BED set at 0 to 5
//添加热床支持只需将#define TEMP_SENSOR_BED 处0设置为5即可

//这里用传感器1来做传感器0的冗余。如果两个传感器温度差较大，将停止打印。
// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT  //设置传感器1作为冗余传感器。
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10, //设置温度最大差值 // Set the maximum temperature difference

// 打印之前通过M109检查当前温度已经接近设置温度，并等待N秒作为缓冲。
// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // 设置达到设置温度后等待时间，单位秒 // (seconds)
#define TEMP_HYSTERESIS 3       //设置离设置温度的浮动范围  // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

//最低温度低于N时，加热头将不会工作。该功能确保温度传感器连接或配置错误时不会烧毁设备。
//检查热敏电阻是否正常。
//如果热门电阻工作不正常，将使加热头电源一直工作。这是非常危险的。
// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5 //设置加热头0的最小温度，一般设置成室内最低温度比较好。因为开机时应该测量到的是室温。 //Set the minimum heating temperature of the head 0, generally set the minimum indoor temperature better. Because the boot should be measured at room temperature.
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

//当温度超过最大设置值，加热头会自动关闭。
//该项配置是为了保护你的设备，避免加热温度过高产生以外。但不能防止温度传感器非正常工作的情况。
//你应该使用MINTEMP选项来保证温度传感器短路或损坏时的设备安全。
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275 //挤出头0 最大保护温度 // Maximum protection temperature extrusion head 0
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define BED_MAXTEMP 120 //热床最大保护温度 // Hot bed temperature for maximum protection

//如果你的热床电流较大，你可以通过设置占空比的方式降低电流，这个值应该是个整数，数字越大，电流越小。
// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

//如果你想用M105命令来显示加热器的功耗，需要设置下面两个参数
// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) EXTRUDER_WATTS
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R

//PID设置
// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#ifdef PIDTEMP
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  #define PID_FUNCTIONAL_RANGE 10  // If the temperature difference between the target temperature and the actual temperature
                                  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_INTEGRAL_DRIVE_MAX 255 //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID
  #define PID_dT ((OVERSAMPLENR * 8.0)/(F_CPU / 64.0 / 256.0))  //sampling period of the temperature routine
  // If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
  // Ultimaker
    #define  DEFAULT_Kp 22.2
    #define  DEFAULT_Ki 1.08
    #define  DEFAULT_Kd 114
    // MakerGear
    //    #define  DEFAULT_Kp 7.0
    //    #define  DEFAULT_Ki 0.1
    //    #define  DEFAULT_Kd 12
    
    // Mendel Parts V9 on 12V
    //    #define  DEFAULT_Kp 63.0
    //    #define  DEFAULT_Ki 2.25
    //    #define  DEFAULT_Kd 440
#endif // PIDTEMP

// Bed Temperature Control
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
//
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT above is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
//#define PIDTEMPBED
//
//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)

#define MAX_BED_POWER 255 //通过占空比方式限制热床的最大功率，255表示不限制 //Limited by the duty cycle Hot-bed maximum power of 255 means no limit
//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#ifdef PIDTEMPBED
    #define  DEFAULT_bedKp 10.00
    #define  DEFAULT_bedKi .023
    #define  DEFAULT_bedKd 305.4
    //120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
    //from pidautotune
    //    #define  DEFAULT_bedKp 97.1
    //    #define  DEFAULT_bedKi 1.41
    //    #define  DEFAULT_bedKd 1675.16
    // FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#endif // PIDTEMPBED

//为了防止加热头未开启时的冷挤出，这里设置当加热头温度未达到N时不允许挤出操作执行。（M302指令可以解除冷挤出限制）//In order to prevent the thermal head is not turned on when cold extrusion, 
                                                                                                  //where the head is set when the heating temperature does not reach N allowed extrusion operation is performed. 
                                                                                                  //(M302 instruction can relieve cold extrusion limit)
//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 175//设置挤出头运行的最低温度 
                           // Setting the minimum temperature extrusion head running, may want ot change this too
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //避免非常长的挤出操作 //prevent extrusion of very large distances.


/*================== Thermal Runaway Protection ==============================
This is a feature to protect your printer from burn up in flames if it has
a thermistor coming off place (this happened to a friend of mine recently and
motivated me writing this feature).

The issue: If a thermistor come off, it will read a lower temperature than actual.
The system will turn the heater on forever, burning up the filament and anything
else around.

After the temperature reaches the target for the first time, this feature will 
start measuring for how long the current temperature stays below the target 
minus _HYSTERESIS (set_temperature - THERMAL_RUNAWAY_PROTECTION_HYSTERESIS).

If it stays longer than _PERIOD, it means the thermistor temperature
cannot catch up with the target, so something *may be* wrong. Then, to be on the
safe side, the system will he halt.

Bear in mind the count down will just start AFTER the first time the 
thermistor temperature is over the target, so you will have no problem if
your extruder heater takes 2 minutes to hit the target on heating.

*/
// If you want to enable this feature for all your extruder heaters,
// uncomment the 2 defines below:

// Parameters for all extruder heaters
#define THERMAL_RUNAWAY_PROTECTION_PERIOD 40 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_HYSTERESIS 4 // in degree Celsius

// If you want to enable this feature for your bed heater,
// uncomment the 2 defines below:

// Parameters for the bed heater
#define THERMAL_RUNAWAY_PROTECTION_BED_PERIOD 20 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS 2 // in degree Celsius
//===========================================================================



//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Uncomment the following line to enable CoreXY kinematics
// #define COREXY //取消前面的注释可以期待用corexy运动系统

// coarse Endstop Settings
// 限位开关设置
#define ENDSTOPPULLUPS  //将上面参数用“//”注释掉，将禁用限位开关的上拉电阻。该配置是全局配置，不用该参数可以用下面单独设置是否开启上拉电阻
                        // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
 //分别对限位开关单独设置上拉电阻。如果ENDSTOPPULLUPS被定义，该配置将被忽略
 // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  // #define ENDSTOPPULLUP_ZMAX
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
//如果你使用机械式的限位开关，并且接到了信号和GND两个接口，那么上面的上拉配置需要打开
//配置3个轴的限位开关类型的，配置为true，限位开关应该接常开端子。如果你接常闭端子，则将true改为false
//设置为true来颠倒限位开关逻辑值。如果设置为true时，限位开关实际的开/合与检测相反，则将该参数配置为false
// translation: 
// If you use a mechanical limit switches, and two received signals and GND interface, then pull on the above configuration requires open
// Configure the limit switch type 3-axis, and is configured as true, limit switches should be connected to the normally open terminal. If you take the normally closed terminal, will be true to false
// Set to true to reverse limit switch logic value. If set to true, the actual limit switch opening / closing and testing the contrary, this parameter is configured as false

const bool X_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = false; 
const bool Z_MIN_ENDSTOP_INVERTING = false;
const bool X_MAX_ENDSTOP_INVERTING = false; 
const bool Y_MAX_ENDSTOP_INVERTING = false;
const bool Z_MAX_ENDSTOP_INVERTING = false; 
//#define DISABLE_MAX_ENDSTOPS
//#define DISABLE_MIN_ENDSTOPS

//为了挡块检查程序的兼容性禁用最大终点挡块
// Disable max endstops for compatibility with endstop checking routine
#if defined(COREXY) && !defined(DISABLE_MAX_ENDSTOPS)
  #define DISABLE_MAX_ENDSTOPS
#endif

//设置步进电机使能引脚的电平。（4988模块保持0即可）
// translation: // Set the stepping motor enable pin level. (4988 module can be kept 0)
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // 针对所有挤出机有效 // Valid for all extruder

//当哪个轴不运动时是否关闭电机。（注意：如果这里打开将会使电机在不使用时被锁止，而导致电机温度急剧上升）
// translation: 
// Which axis is not moving when the motor is turned off. (Note: If you will be here to open the motor is locked when not in use, resulting in a sharp rise in motor temperature)
// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false //针对所有挤出机有效 // For all extruders

//电机运动方向控制。由于电机连线不同，电机的运动方向也不同，但打印机的0点位置在左下角，如果电机的运动方向
//与控制方向不同，则可以将下面参数值true和false对调，也可以将步进电机的4根线反过来插。
// Direction of motor motion control. Due to the different electrical connections, the direction of movement of the motor is different, but the 0:00 position of the printer in the lower left corner, if the direction of motion of the motor
// Different direction and control, you can place the following parameter values true and false reversed, may be 4-wire stepper motor is in turn inserted.
#define INVERT_X_DIR true    // (X轴配置）: (X-axis configuration)// for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR true    // (Y轴配置）: (Y-axis configuration)
#define INVERT_Z_DIR true    // (Z轴配置）: (Y-axis configuration)
#define INVERT_E0_DIR false   //  (挤出机0配置）: (0 extruder configuration) // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false   // (挤出机1配置）: (Extruder 1 configuration) // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // (挤出机2配置）: (2 extruder configuration) // for direct drive extruder v9 set to true, for geared extruder set to false

//停止开关设置
//设置回0时，电机的运动方向。1最大限位方向，-1最小限位方向。一般都是设置为-1
// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1
//软限位开关设置
// Software limit switch)
#define min_software_endstops false //最小值设置，如果设置为true，则移动距离<HOME_POS值 // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  //最大值设置，如果设置为true，轴不会移动到坐标大于下面定义的长度。 // If true, axis won't move to coordinates greater than the defined lengths below.

//各轴的软件限位值
// Travel limits after homing
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define X_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)


//================= MICROMAKE 3D打印机 自动调平配置 ====================
//============================= Bed Auto Leveling ===========================
#define ENABLE_AUTO_BED_LEVELING // 是否开启自动调平功能 
                                 // Delete the comment to enable (remove // at the start of the line)

#ifdef ENABLE_AUTO_BED_LEVELING
// There are 2 different ways to pick the X and Y locations to probe:

//  - "grid" mode
//    Probe every point in a rectangular grid
//    You must specify the rectangle, and the density of sample points
//    This mode is preferred because there are more measurements.
//    It used to be called ACCURATE_BED_LEVELING but "grid" is more descriptive

//  - "3-point" mode
//    Probe 3 arbitrary points on the bed (that aren't colinear)
//    You must specify the X & Y coordinates of all 3 points


  #define DELTA_PROBABLE_RADIUS (DELTA_PRINTABLE_RADIUS-50)//此处设置为调平探针移动范围，增大调平范围减少“-50”这个值，减少调平范围增大“-50”这个值
                                                           //// Here to leveling the moving range of the probe, increasing the range of leveling down "-50" this value is increased to reduce the leveling range "-50" value
  //如：#define DELTA_PROBABLE_RADIUS (DELTA_PRINTABLE_RADIUS-60)
  
  #define LEFT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS
  #define RIGHT_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
  #define BACK_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
  #define FRONT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS

// these are the offsets to the probe relative to the extruder tip (Hotend - Probe)
/* from original marlin
 *  #define X_PROBE_OFFSET_FROM_EXTRUDER -22.919
  #define Y_PROBE_OFFSET_FROM_EXTRUDER -6.304
  #define Z_PROBE_OFFSET_FROM_EXTRUDER -17.25  // Increase this if the first layer is too thin.
  */
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0.0
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0.0
  #define Z_PROBE_OFFSET_FROM_EXTRUDER 1.6//自动调平设置 过高减小 过低增大
                                          //translation: Automatic leveling set too high too low to reduce the increase

  #define Z_RAISE_BEFORE_HOMING 4       // 配置回原点前Z轴升起的高度，该高度要确保在Z轴最大高度范围内。 
                                        // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case
  
  #define XY_TRAVEL_SPEED 2000         //执行自动调平移动的速度，增大速度增加，减小速度降低
                                        // X and Y axis travel speed between probes, in mm/min

  #define Z_RAISE_BEFORE_PROBING 80  ////经过第一个检测点前Z轴抬起的高度，该高度要确保调平传感器可以正常放下。
                                     //How much the extruder will be raised before traveling to the first probing point.
  #define Z_RAISE_BETWEEN_PROBINGS 5  //经过下一个检测点前Z轴抬起的高度
                                      //How much the extruder will be raised when traveling from between next probing points

//If you have enabled the Bed Auto Leveling and are using the same Z Probe for Z Homing,
//it is highly recommended you let this Z_SAFE_HOMING enabled!!!

  #define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with probe outside the bed area.
                          // When defined, it will:
                          // - Allow Z homing only after X and Y homing AND stepper drivers still enabled
                          // - If stepper drivers timeout, it will need X and Y homing again before Z homing
                          // - Position the probe in a defined XY point before Z Homing when homing all axis (G28)
                          // - Block Z homing only when the probe is outside bed area.
  #ifdef Z_SAFE_HOMING

    #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)   // X point for Z homing when homing all axis (G28)
    #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)   // Y point for Z homing when homing all axis (G28)

  #endif


  #define ACCURATE_BED_LEVELING
  #ifdef ACCURATE_BED_LEVELING
    #define ACCURATE_BED_LEVELING_POINTS 3 //自动调平探头点点数 3为横竖向各点3个点，共9点，改为6就是横竖向各点6个点，共36个点。
                                           //Automatic leveling, vertical probe points to 3 points each by 3 points, a total of nine points to six points is horizontal, six points each, a total of 36 points.
    #define ACCURATE_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1))
    #define ACCURATE_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1))
    #define NONLINEAR_BED_LEVELING
  #endif

#endif  // ENABLE_AUTO_BED_LEVELING

//归位开关设置
// The position of the homing switches
#define MANUAL_HOME_POSITIONS  //如果开启该配置，下面 MANUAL_*_HOME_POS配置将生效
                                 // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0  //如果开启该配置，热床的中心位置在(X=0, Y=0) 
                             // If defined, the center of the bed is at (X=0, Y=0)

//手动回零开关的位置：
//对于三角洲结构这意味着笛卡尔打印机的顶部和中心的值。
//Manual homing switch locations:
// For deltabots this means top and center of the Cartesian print volume.
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 185.9 // no heat bed: 206.6 
                                // real, no heat bed: 210.0//306.6 
                                // Z轴高度设置 
                                // was 230
//因每台机器安装会有差别，需自行测量，测量方法请查看配套视频教程，设置完后后记得保持修改
// translation: // Because each machine installed there will be differences, should make their own measurements, measuring methods please see supporting video tutorials, after holding modify settings after recall

//轴设置
//// MOVEMENT SETTINGS
#define NUM_AXIS 4 //轴的数量，各轴的配置是顺序是X, Y, Z, E 
                   // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {60*60, 60*60, 60*60, 0}  //配置归位时的速度
                                                   // set the homing speeds (mm/min)
// default settings
#define XYZ_FULL_STEPS_PER_ROTATION 200 //步进电机每周的步数，即360/步进电机上的角度
                                        // translation: // Stepper motor weekly number of steps, that is, the angle of 360 / stepper motor on)
//如1.8度，步数应该是360/1.8=200；如果是0.9度电机的话就是 360/0.9=400。27号以前购买的用户请修改为400，27号以后的用户请修改为200。
// translation: // As 1.8 degrees, the number of steps should be 360 / 1.8 = 200; if it is 0.9 degrees, then the motor is 360 / 0.9 = number 400.27 previously purchased by the user to modify the number of users, please 400,27 later revised to 200.
#define XYZ_MICROSTEPS 16 //步进驱动的细分数 : // Number of stepper drive segment
#define XYZ_BELT_PITCH 2 //同步带齿间距     : // Belt tooth pitch
#define XYZ_PULLEY_TEETH 16 //同步轮齿数   : // Synchronization number of teeth
#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION * XYZ_MICROSTEPS / double(XYZ_BELT_PITCH) / double(XYZ_PULLEY_TEETH))
//这是计算公式：步进电机数*步进驱动的细分数/同步带齿间距/同步轮齿数
// translation: // This is the formula: number of subdivisions stepper motor drive stepper number * / belt tooth pitch / number of teeth synchronous)

#define DEFAULT_AXIS_STEPS_PER_UNIT   {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, 98}   //挤出机挤出量: // Extruder amount
#define DEFAULT_MAX_FEEDRATE          {200, 200, 200, 200}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {3000,3000,3000,3000}   // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.


//加速度配置，假如打印时失步太大，可以将这个值改小
// translation: // Acceleration configuration, when out of step if the print is too large, this value can be piecemeal
#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000    // X, Y, Z and E max acceleration in mm/s^2 for retracts

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
//各轴不需要加速的距离，即无需加速，立即完成的距离（即软件认为他可以在瞬间完成的）
#define DEFAULT_XYJERK                20.0   // (mm/sec)
#define DEFAULT_ZJERK                 20.0    
#define DEFAULT_EJERK                 20.0  

//===========================================================================
//===============================附加功能====================================
//===========================================================================
//
//===========================================================================
//=============================Additional Features===========================
//===========================================================================
//以下内容暂未汉化，后续会持续汉化更新，欢迎加入我们的交流QQ群：224981313
//感谢您的支持，也欢迎更多朋友可以持续补充，完善，欢迎散播复制，尊重劳动者，使用发布请注明出处。
// translation
// Write the following finished, follow-up will continue to update finished, welcome to join our exchange QQ group: 224 981 313
// Thank you for your support, and also welcomes more friends can continue to add and improve, welcome to spread replication, respect for workers, using publications please indicate the source.
//
// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
//#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

//LCD and SD support
//#define ULTRA_LCD  //general LCD support, also 16x2
//#define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
//#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define ENCODER_PULSES_PER_STEP 1 // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5 // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.
//#define ULTIPANEL  //the UltiPanel as on Thingiverse
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000	// this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click

// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARUDINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

//automatic expansion
#if defined (MAKRPANEL)
 #define DOGLCD
 #define SDSUPPORT
 #define ULTIPANEL
 #define NEWPANEL
 #define DEFAULT_LCD_CONTRAST 17
#endif

#if defined (REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
 #define DOGLCD
 #define U8GLIB_ST7920
 #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if defined(ULTIMAKERCONTROLLER) || defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
#endif

#if defined(REPRAPWORLD_KEYPAD)
  #define NEWPANEL
  #define ULTIPANEL
#endif
#if defined(RA_CONTROL_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
 #define LCD_I2C_TYPE_PCA8574
 #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
#endif

//I2C PANELS

//#define LCD_I2C_SAINSMART_YWROBOT
#ifdef LCD_I2C_SAINSMART_YWROBOT
  // This uses the LiquidCrystal_I2C library ( https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home )
  // Make sure it is placed in the Arduino libraries directory.
  #define LCD_I2C_TYPE_PCF8575
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define NEWPANEL
  #define ULTIPANEL
#endif

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//#define LCD_I2C_PANELOLU2
#ifdef LCD_I2C_PANELOLU2
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // (v1.2.3 no longer requires you to define PANELOLU in the LiquidTWI2.h library header file)
  // Note: The PANELOLU2 encoder click input can either be directly connected to a pin
  //       (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD
  #define NEWPANEL
  #define ULTIPANEL

  #ifndef ENCODER_PULSES_PER_STEP
	#define ENCODER_PULSES_PER_STEP 4
  #endif

  #ifndef ENCODER_STEPS_PER_MENU_ITEM
	#define ENCODER_STEPS_PER_MENU_ITEM 1
  #endif


  #ifdef LCD_USE_I2C_BUZZER
	#define LCD_FEEDBACK_FREQUENCY_HZ 1000
	#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif

#endif

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI
#ifdef LCD_I2C_VIKI
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // Note: The pause/stop/resume LCD button pin should be connected to the Arduino
  //       BTN_ENC pin (or set BTN_ENC to -1 if not used)
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define NEWPANEL
  #define ULTIPANEL
#endif

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
//#define SR_LCD
#ifdef SR_LCD
   #define SR_LCD_2W_NL    // Non latching 2 wire shift register
   //#define NEWPANEL
#endif


#ifdef ULTIPANEL
//  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the DOG graphic display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif
#else //no panel but just LCD
  #ifdef ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the 128x64 graphics display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 16
    #define LCD_HEIGHT 2
  #endif
  #endif
#endif

// default LCD contrast for dogm-like LCD displays
#ifdef DOGLCD
# ifndef DEFAULT_LCD_CONTRAST
#  define DEFAULT_LCD_CONTRAST 32
# endif
#endif

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Temperature status LEDs that display the hotend and bet temperature.
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
// Otherwise the RED led is on. There is 1C hysteresis.
//#define TEMP_STAT_LEDS

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
// #define PHOTOGRAPH_PIN     23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.
//#define BARICUDA

//define BlinkM/CyzRgb Support
//#define BLINKM

/*********************************************************************\
* R/C SERVO support
* Sponsored by TrinityLabs, Reworked by codexmas
**********************************************************************/

// Number of servos
//
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it undefined or defining as 0 will disable the servo subsystem
// If unsure, leave commented / disabled
//
//#define NUM_SERVOS 3 // Servo index starts with 0 for M280 command

// Servo Endstops
//
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M206 command to correct for switch height offset to actual nozzle height. Store that setting with M500.
//
//#define SERVO_ENDSTOPS {-1, -1, 0} // Servo index for X, Y, Z. Disable with -1
//#define SERVO_ENDSTOP_ANGLES {0,0, 0,0, 70,0} // X,Y,Z Axis Extend and Retract angles

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
