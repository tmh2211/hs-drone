module Robotics.ArDrone.Repl.ReplCommands where

import Text.Read (readMaybe)

data ReplCommand = Initialize
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


readReplCommand :: String -> Maybe ReplCommand
readReplCommand = readMaybe
