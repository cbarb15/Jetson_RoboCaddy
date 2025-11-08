import time

import bluetooth
from gi.repository import GLib
import bluetooth_utils
import bluetooth_constants
import dbus
import dbus.mainloop.glib
import sys
import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice
import multiprocessing
import serial

sys.path.insert(0, '.')

SERIAL_PORT= "/dev/ttyTHS1"   # Jetson Orin Nano
BAUD_RATE = 115200
uart = None
adapter_interface = None
mainloop = None
timer_id = None
bus = None
device_interface = None
device_path = None
found_dis = False
found_mn  = False
dis_path = None
mn_path  = None
# ADDED
found_ts = False
found_tc = False
ts_path = None
tc_path = None

devices = {}
managed_objects_found = 0
reading_process = None

def send_joystick_data(joystick_value):
    try:
        print(f"Serial port {SERIAL_PORT} opened successfully.")
        # num = 3127
        uart.write(joystick_value.to_bytes(2, byteorder='big'))
        uart.flush()
        print(f"Sent: {joystick_value}")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")

def joystick_read_process():
    while 1:
        read_joystick()


def read_joystick():
    global tc_path
    char_proxy = bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME,tc_path)
    char_interface = dbus.Interface(char_proxy,
    bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE)
    try:
        value = char_interface.ReadValue({})
        print(f"type of value {type(value)}")
    except Exception as e:
        print("Failed to read joystick")
        print(e.get_dbus_name())
        print(e.get_dbus_message())
        return bluetooth_constants.RESULT_EXCEPTION
    else:
        joystick = bluetooth_utils.dbus_to_python(int.from_bytes(value, 'big'))
        # print("Joystick value: "+str(joystick))
        send_joystick_data(joystick)
        return bluetooth_constants.RESULT_OK


def service_discovery_completed():
    global found_ts
    global found_tc
    global ts_path
    global tc_path
    global bus

    if found_ts and found_tc:
        print("Required service and characteristic found - device is OK")
        print("Joystick service path: ", ts_path)
        print("Joystick characteristic path: ", tc_path)
        reading_process.start()
    else:
        print("Required service and characteristic were not found - device is NOK")
        print("Joystick service found: ", str(found_ts))
        print("Joystick characteristic found: ", str(found_tc))
    bus.remove_signal_receiver(interfaces_added, "InterfacesAdded")
    bus.remove_signal_receiver(properties_changed, "PropertiesChanged")
    mainloop.quit()


def list_devices_found():
    print("Full list of devices", len(devices), "discovered:")
    print("------------------------------")
    for path in devices:
        dev = devices[path]
        print(bluetooth_utils.dbus_to_python(dev['Address']))


def interfaces_added(path, interfaces):
    global found_ts
    global found_tc
    global ts_path
    global tc_path
    if bluetooth_constants.GATT_SERVICE_INTERFACE in interfaces:
        properties = interfaces[bluetooth_constants.GATT_SERVICE_INTERFACE]
        print("--------------------------------------------------------------------------------")
        print("SVC path   :", path)
        if 'UUID' in properties:
            uuid = properties['UUID']
            if uuid == bluetooth_constants.JOYSTICK_SVC_UUID:
                found_ts = True
                ts_path = path
            print("SVC UUID   : ", bluetooth_utils.dbus_to_python(uuid))
            print("SVC name   : ", bluetooth_utils.get_name_from_uuid(uuid))
        return

    if bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE in interfaces:
        properties = interfaces[bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE]
        print("  CHR path   :", path)
        if 'UUID' in properties:
            uuid = properties['UUID']
            if uuid == bluetooth_constants.JOYSTICK_CHR_UUID:
                found_tc = True
                tc_path = path
            print("  CHR UUID   : ", bluetooth_utils.dbus_to_python(uuid))
            print("  CHR name   : ", bluetooth_utils.get_name_from_uuid(uuid))
            flags = ""
            for flag in properties['Flags']:
                flags = flags + flag + ","
            print("  CHR flags  : ", flags)
        return

    if bluetooth_constants.GATT_DESCRIPTOR_INTERFACE in interfaces:
        properties = interfaces[bluetooth_constants.GATT_DESCRIPTOR_INTERFACE]
        print("    DSC path   :", path)
        if 'UUID' in properties:
            uuid = properties['UUID']
            print("    DSC UUID   : ", bluetooth_utils.dbus_to_python(uuid))
            print("    DSC name   : ", bluetooth_utils.get_name_from_uuid(uuid))
        return


def interfaces_removed(path, interfaces):
    # interfaces is an array of dictionary strings in this signal
    if not bluetooth_constants.DEVICE_INTERFACE in interfaces:
        return
    if path in devices:
        dev = devices[path]
        if 'Address' in dev:
            print("DEL bdaddr: ", bluetooth_utils.dbus_to_python(dev['Address']))
        else:
            print("DEL path  : ", path)
            print("------------------------------")
        del devices[path]


def properties_changed(interface, changed, invalidated, path):
    global device_path
    if path != device_path:
        return

    if 'ServicesResolved' in changed:
        sr = bluetooth_utils.dbus_to_python(changed['ServicesResolved'])
        print("ServicesResolved  : ", sr)
        if sr == True:
            service_discovery_completed()


def get_known_devices(bus):
    global managed_objects_found
    object_manager = dbus.Interface(bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME, "/"),
                                    bluetooth_constants.DBUS_OM_IFACE)
    managed_objects = object_manager.GetManagedObjects()

    for path, ifaces in managed_objects.items():
        for iface_name in ifaces:
            if iface_name == bluetooth_constants.DEVICE_INTERFACE:
                managed_objects_found += 1
                print("EXI path  : ", path)
                device_properties = ifaces[bluetooth_constants.DEVICE_INTERFACE]
                devices[path] = device_properties
                if 'Address' in device_properties:
                    print("EXI bdaddr: ", bluetooth_utils.dbus_to_python(device_properties['Address']))
                if 'Connected' in device_properties:
                    print("EXI cncted: ", bluetooth_utils.dbus_to_python(device_properties['Connected']))
                print("------------------------------")


def discovery_timeout():
    global adapter_interface
    global mainloop
    global timer_id
    GLib.source_remove(timer_id)
    mainloop.quit()
    adapter_interface.StopDiscovery()
    bus = dbus.SystemBus()
    bus.remove_signal_receiver(interfaces_added, "InterfacesAdded")
    bus.remove_signal_receiver(interfaces_added, "InterfacesRemoved")
    bus.remove_signal_receiver(properties_changed, "PropertiesChanged")
    list_devices_found()
    return True


def discover_devices(bus, timeout):
    global adapter_interface
    global mainloop
    global timer_id
    adapter_path = bluetooth_constants.BLUEZ_NAMESPACE + bluetooth_constants.ADAPTER_NAME

    # acquire the adapter interface so we can call its methods
    adapter_object = bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME, adapter_path)
    adapter_interface = dbus.Interface(adapter_object, bluetooth_constants.ADAPTER_INTERFACE)

    # register signal handler functions so we can asynchronously report discovered devices

    # InterfacesAdded signal is emitted by BlueZ when an advertising packet from a device it doesn't
    # already know about is received
    bus.add_signal_receiver(interfaces_added,
                            dbus_interface=bluetooth_constants.DBUS_OM_IFACE,
                            signal_name="InterfacesAdded")

    # InterfacesRemoved signal is emitted by BlueZ when a device "goes away"
    bus.add_signal_receiver(interfaces_removed,
                            dbus_interface=bluetooth_constants.DBUS_OM_IFACE,
                            signal_name="InterfacesRemoved")

    # PropertiesChanged signal is emitted by BlueZ when something re: a device already encountered
    # changes e.g. the RSSI value
    bus.add_signal_receiver(properties_changed,
                            dbus_interface=bluetooth_constants.DBUS_PROPERTIES,
                            signal_name="PropertiesChanged",
                            path_keyword="path")

    mainloop = GLib.MainLoop()
    timer_id = GLib.timeout_add(timeout, discovery_timeout)
    adapter_interface.StartDiscovery(byte_arrays=True)

    mainloop.run()

    device_list = devices.values()
    discovered_devices = []
    for device in device_list:
        dev = {}
        discovered_devices.append(dev)

    return discovered_devices

def is_connected(device_proxy):
    global bus
    props_interface = dbus.Interface(device_proxy, bluetooth_constants.DBUS_PROPERTIES)
    connected = props_interface.Get(bluetooth_constants.DEVICE_INTERFACE,"Connected")
    return connected

def connect():
    global bus
    global device_interface
    try:
        device_interface.Connect()
    except Exception as e:
        print("Failed to connect")
        print(e.get_dbus_name())
        print(e.get_dbus_message())
        if ("UnknownObject" in e.get_dbus_name()):
            print("Try scanning first to resolve this problem")
        return bluetooth_constants.RESULT_EXCEPTION
    else:
        print("Connected OK")
        return bluetooth_constants.RESULT_OK

def disconnect():
    global bus
    global device_interface
    try:
        device_interface.Disconnect()
    except Exception as e:
        print("Failed to disconnect")
        print(e.get_dbus_name())
        print(e.get_dbus_message())
        return bluetooth_constants.RESULT_EXCEPTION
    else:
        print("Disconnected OK")
        return bluetooth_constants.RESULT_OK

if __name__ == '__main__':
    scantime = 5 * 1000

    try:
        uart = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")

    # dbus initialisation steps
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    bdaddr = "D8:3A:DD:CA:6E:3A"

    # ask for a list of devices already known to the BlueZ daemon
    print("Listing devices already known to BlueZ:")
    get_known_devices(bus)
    print("Found ", managed_objects_found, " managed device objects")
    print("Scanning")
    discover_devices(bus, scantime)

    adapter_path = bluetooth_constants.BLUEZ_NAMESPACE + bluetooth_constants.ADAPTER_NAME
    device_path = bluetooth_utils.device_address_to_path(bdaddr, adapter_path)
    device_proxy = bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME, device_path)
    device_interface = dbus.Interface(device_proxy, bluetooth_constants.DEVICE_INTERFACE)
    reading_process = multiprocessing.Process(target=joystick_read_process)

    time.sleep(2)
    print("Connecting to " + bdaddr)
    connect()
    print("Discovering services++")
    print("Registering to receive InterfacesAdded signals")
    bus.add_signal_receiver(interfaces_added,
                            dbus_interface=bluetooth_constants.DBUS_OM_IFACE,
                            signal_name="InterfacesAdded")
    print("Registering to receive PropertiesChanged signals")
    bus.add_signal_receiver(properties_changed,
                            dbus_interface=bluetooth_constants.DBUS_PROPERTIES,
                            signal_name="PropertiesChanged",
                            path_keyword="path")
    mainloop = GLib.MainLoop()
    mainloop.run()
    # print("Finished")
    # disconnect()