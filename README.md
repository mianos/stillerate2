# Reflux Still Control


# Examples

Set the reflux pump url

```
mosquitto_pub -h mqtt2.mianos.com -t "cmnd/still2/settings" -m '{"refluxPumpUrl": "http://reflux/pump"}'
```

Set the device host name
```
 mosquitto_pub -h mqtt2.mianos.com -t "cmnd/still2/wificonfig" -m '{"host_name": "still2"}'
```

# TODO
 examples for settings
 start stop control
 runtime posts
