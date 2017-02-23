{-|
Module:         Robotics.ArDrone.NavDataConstants
Description:    NavDataOption to number conversion
License:        GPL-2
Maintainer:     thomasmhergenroeder@gmail.com
Stability:      Tested on Windows 10

This module provides a data type with a value for each possible option the drone
offers. Also a function that converts that type to a number, so the user has
not to worry about implementation details and bit numbers, when configuring the
navdata options.
-}
module Robotics.ArDrone.NavDataConstants where

-- | Data type that provides easy readable values for each navdata option
data NavDataOption = DEMO
                    | TIME
                    | RAW_MEASURES
                    | PHYS_MEASURES
                    | GYROS_OFFSETS
                    | EULER_ANGLES
                    | REFERENCES
                    | TRIMS
                    | RC_REFERENCES
                    | PWM
                    | ALTITUDE
                    | VISION_RAW
                    | VISION_OF
                    | VISION
                    | VISION_PERF
                    | TRACKERS_SEND
                    | VISION_DETECT
                    | WATCHDOG
                    | ADC_DATA_FRAME
                    | VIDEO_STREAM
                    | GAMES
                    | PRESSURE_RAW
                    | MAGNETO
                    | WIND_SPEED
                    | KALMAN_PRESSURE
                    | HDVIDEO_STREAM
                    | WIFI
                    | ZIMMU_3000
                    | CKS

-- | Function that converts a NavDataOption value to a number to simplify the
-- conversion in the navigational data configuration.
optionToInt :: NavDataOption -> Int
optionToInt DEMO = 0
optionToInt TIME = 1
optionToInt RAW_MEASURES = 2
optionToInt PHYS_MEASURES = 3
optionToInt GYROS_OFFSETS = 4
optionToInt EULER_ANGLES = 5
optionToInt REFERENCES = 6
optionToInt TRIMS = 7
optionToInt RC_REFERENCES = 8
optionToInt PWM = 9
optionToInt ALTITUDE = 10
optionToInt VISION_RAW = 11
optionToInt VISION_OF = 12
optionToInt VISION = 13
optionToInt VISION_PERF = 14
optionToInt TRACKERS_SEND = 15
optionToInt VISION_DETECT = 16
optionToInt WATCHDOG = 17
optionToInt ADC_DATA_FRAME = 18
optionToInt VIDEO_STREAM = 19
optionToInt GAMES = 20
optionToInt PRESSURE_RAW = 21
optionToInt MAGNETO = 22
optionToInt WIND_SPEED = 23
optionToInt KALMAN_PRESSURE = 24
optionToInt HDVIDEO_STREAM = 25
optionToInt WIFI = 26
optionToInt ZIMMU_3000 = 27
optionToInt CKS = 65535
