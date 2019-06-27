H4dump
-----------------------

### Install

```
cmake -B build
cmake --buld build
cp build/btuart $HOME/.local/bin
```

### Usage
```
usage: btuart -w {btsnoop} -r {rx} -t {tx} -b {baudrate} -h
	-w btsnoop	caputre data write to the btsnoop file
	-r tty		H4 RX connect device (eg: /dev/ttyUSB0)
	-t tty		H4 TX connect device (eg: /dev/ttyUSB1)
	-b baudrate	H4 uart buadrate
```

### Hardware connection
(BT H4 RX)	--------------->	(FT232 or other uart device 1 RX)
(BT H4 TX)	---------------> 	(FT232 or other uart device 2 RX)
