#!/bin/expect -f

set password "000000"

# Change permissions for /dev/Arduino
spawn sudo chmod 777 /dev/Arduino
expect "assword for"
send "$password\r"
expect eof

# Change permissions for /dev/RosMasterBoard
spawn sudo chmod 777 /dev/RosMasterBoard
expect "assword for"
send "$password\r"
expect eof

# Change permissions for /dev/Lidar
spawn sudo chmod 777 /dev/Lidar
expect "assword for"
send "$password\r"
expect eof