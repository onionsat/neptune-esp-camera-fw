# Neptune ESP Camera firmware documentation

## I2C commands
| Commands | Command code | Return value |
| ------------- | ------------- | ------------- |
| Get error  | 0x01  | 1 byte error code |

### Get error
Returns a 8 bit error code, where each bit represents a separate error. When the bit is 1, the error is present.
| Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
| :-----: | :-----: | :-----: | :-----: | :-----: | :-----: | :-----: | :-----: |
| - | - | - | Write error fail | Frame grab fail | File open fail | SD card mount fail | Camera init fail |