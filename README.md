#
#       Author: Vincent Gijsen
#

# Background milight connections
working clock settings for LT8900:

0.24ms, 2khz

pinout on MiLight hardware (led driver board for external strip)
```
pl11167 --------------|| -------------------- stm8s003

pin 
1 -> avss 
2 -> nc
3 -> nc
4 -> pkt        ||              15 pc5
5 -> rstb       ||              gnd
6-> dvss                        3 pd6
7-> scsb        ||              1 pd4
8 -> sck        ||              2 pd5

16-> ant
15-> antb
14 -> xin
13 -> xout
12 -> vdd0
11 -> vcc
10 -> sdo       ||              6 PA2
09 -> sdi              ||       20 pd3

-------------------------------------------------------

                                STM8S03 ->led

                                17 -> Red mosfet
                                16 -> Green mosfet
                                13 -> Blue mosfet
                                14 -> White
``` 
#
# 3W bulb
#
```
pl1156 --- smt8


4               3
5               15
7               20
8                19
9               1
10              6
```
# Alternative pin settings

stm pins:
```
C6 (pin 16): TIM1_CH1
C7 (pin 17): TIM1_CH2
C3 (pin 13): TIM1_CH3
C4 (pin 14): TIM1_CH4
```                               


SET AFR0 to 1 in OPTIONBYTE2
as to datasheet:
```
C6 alternate function =
TIM1_CH1; port C7 alternate function = TIM1_CH2.
```

# Packet format

## First byte:
nodeID;
0; reserved for the master node...
1-254: nodes

## second byte: 
````
packet type
0: presense
1: set cmd
2: request-last-statue
```


## present package
```
3rd byte:
0; rgbw
1; rgb
2; white
3; ..?
```
node | 0 | RGBW =0

## set cmd
```
nodeId | 1 | r | g | b | w| <time>
<time>
0: immidiateley
1 - 255 : steps in ms for each 'step' from current to next value
```
## RequestState package
nodeId | 2

