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
usage: btuart -w {btsnoop} -r {rx} -t {tx} -b {baudrate} -5 -4 -h
      -w {file}     write the dump data to the {file}
      -r {dev}      monitor the serial port receiving data
      -t {uart}     monitor the serial port that sends data
      -b {baudate}  serial buad rate
      -4            when the captured data is not H4 protocol
                    use H4 protocol to save the data
      -5            use h5 protocol
      -h            display the message

```

### Hardware connection
(BT UART RX)	--------------->	(FT232 or other uart device 1 RX)
(BT UART TX)	---------------> 	(FT232 or other uart device 2 RX)
