bytes 0..12   = IMU
bytes 13..36  = ADS1299 sample 1: CH1..CH8
bytes 37..60  = ADS1299 sample 2
bytes 61..84  = ADS1299 sample 3
bytes 85..109  = ADS1299 sample 4

0
├──────────── IMU ────────────┤
0                           12

13
├── CH1..CH8 ──┤
13               36

37
├── CH1..CH8 ──┤
37             60

61
├──  CH1..CH8 ──┤
61              84


61
├──  CH1..CH8 ──┤
85             109
