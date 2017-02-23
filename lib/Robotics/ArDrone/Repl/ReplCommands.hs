{-|
Module:         Robotics.ArDrone.Repl.ReplCommands
Description:    Functionality to read REPL commands from terminal.
License:        GPL-2
Maintainer:     thomasmhergenroeder@gmail.com
Stability:      Tested on Windows 10
-}
module Robotics.ArDrone.Repl.ReplCommands where
import Text.Read (readMaybe)

-- | Data type to represent the possible commands to send to the drone. Some
-- additional values added, to exit the program and handle the complete absence
-- of a command.
data ReplCommand = None
                 | TakeOff
                 | Land
                 | Stop
                 | Forward Float
                 | Backwards Float
                 | Left Float
                 | Right Float
                 | Clockwise Float
                 | CounterClockwise Float
                 | Up Float
                 | Down Float
                 | Exit deriving (Show, Read)

-- | Function that reads a command from a string. If the value can't be
-- converted to a ReplCommand it returns nothing.
readReplCommand :: String -> Maybe ReplCommand
readReplCommand = readMaybe
