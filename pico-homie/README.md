# pOwnTracks

Pico on Homie

## Parameters

### `devices/xyxyxyxy/gps/csv/set` [true|false|?]
toggles csv payload or human readable
default is human 'false`

### `devices/xyxyxyxy/gps/minDistance/set` [true|false|?]
sets the minimum distance in meters to travel before the next location
publish is triggered ("t":"v")
Default = `100m`

### `devices/xyxyxyxy/gps/maxInterval/set` [true|false|?]
sets the maximum interval before the next location publish is triggered
in seconds ("t":"p")
Default = `3600s`

### `devices/xyxyxyxy/gps/cmd/set` gps
manually triggers a set of GPS data to be send ("t":"m")
