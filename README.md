# TUX500 ble daemon

## Dependencies

This library uses GLib 2.0. Your SDK must have 


## Building the code


To build the code 
```
cmake .
cmake --build .
```



## Bluez documentation

The official Bluez documentation is a bit sparse but can be found here: 
* [Adapter documentation](https://github.com/bluez/bluez/blob/master/doc/adapter-api.txt) (for default_adapter)
* [GATT documentation](https://github.com/bluez/bluez/blob/master/doc/gatt-api.txt) (for service, characteristics and descriptors)
* [Device documentation](https://github.com/bluez/bluez/blob/master/doc/device-api.txt) (for device)
* [Agent documentation](https://github.com/bluez/bluez/blob/master/doc/agent-api.txt) (for agent)

You will notice that most original methods and properties are available in this library. In some cases, some adaptations have been done for convenience.


## Installing on Raspberry Pi
