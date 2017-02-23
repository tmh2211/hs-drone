{-# LANGUAGE NamedFieldPuns #-}
{-|
Module:       Robotics.ArDrone.Control
Description:  Conversion of High level DroneCommands to strings the drone will understand

This module was taken from the original library that I forked from.
https://github.com/osnr/hs-drone
-}
module Robotics.ArDrone.Control
where

import Network.Socket
import Control.Concurrent
import Control.Monad

import Data.List

import Data.Binary.IEEE754
import Data.Int

-- | High level type that describes the possible commands the user can send to
-- the drone.
data ArDroneMsg = TakeOff
                | Land

                | Up Float
                | Down Float

                | Clockwise Float
                | CounterClockwise Float

                | Front Float
                | Back Float

                | MoveLeft Float
                | MoveRight Float

                | Stop
                | FTrim
                | Calibrate

                | Config String String

                | Animate String Int

                | DisableEmergency
                deriving (Show)

-- | Intermediate representation of the messages that are defined in the drone's
-- protocol
data AtCommand = AtRef String
               | AtFTrim
               | AtPCmd { enable :: Bool
                        , pitch :: Float
                        , roll :: Float
                        , gaz :: Float
                        , yaw :: Float }
               | AtConfig String String
               | AtCtrl Int Int
               deriving (Show)

-- | Synonym for the sequencenumber of the commands
type SequenceNumber = Int

-- | Deprecated: Do not use
connectDrone :: String -> IO Socket
connectDrone ip = do
    addrInfos <- getAddrInfo Nothing (Just ip) (Just "5556")
    let serverAddr = head addrInfos

    sock <- socket (addrFamily serverAddr) Datagram defaultProtocol
    connect sock (addrAddress serverAddr)

    return sock

-- | Deprecated: Do not use
runDrone :: String -> [(Int, ArDroneMsg)] -> IO ()
runDrone ip msgs = do
    sock <- connectDrone ip

    let commands :: [(SequenceNumber, AtCommand)]
        commands = zip [1..] . concat $ do
            (waitAfterward, msg) <- msgs
            -- how many times do we send this msg out?
            -- supposing we send it out every 30 ms
            let n = waitAfterward `div` 30

            return . replicate n $ toAtCommand msg

    forM_ commands $ \(num, command) -> do
        send sock $ fromAtCommand command num
        threadDelay 30000

-- | Do not use this function
main = withSocketsDo $ do
    runDrone "192.168.1.1" $
        [ (6000, TakeOff)
        , (2000, CounterClockwise 0.5)
        , (2000, Land)
        ]

-- | This function converts a message like flying forward to the intermediate
-- representation of the message types.
toAtCommand :: ArDroneMsg -> AtCommand
toAtCommand msg =
    case msg of
     TakeOff -> AtRef "290718208"
     Land -> AtRef "290717696"

     Up speed -> AtPCmd True 0 0 speed 0
     Down speed -> AtPCmd True 0 0 (-speed) 0

     -- fixme not sure which is which
     Clockwise speed -> AtPCmd True 0 0 0 speed
     CounterClockwise speed -> AtPCmd True 0 0 0 (-speed)

     Front speed -> AtPCmd True 0 (-speed) 0 0
     Back speed -> AtPCmd True 0 speed 0 0

     MoveLeft speed -> AtPCmd True speed 0 0 0
     MoveRight speed -> AtPCmd True (-speed) 0 0 0

     Stop -> AtPCmd False 0 0 0 0

     FTrim -> AtFTrim
     DisableEmergency -> AtRef "290717952"

-- | This converts the intermediate message representation and a given sequence
-- number to a string the drone will understand.
fromAtCommand :: AtCommand -> Int -> String
fromAtCommand cmd num = (++ "\r") $
   case cmd of
     AtRef param -> "AT*REF=" ++ show num ++ "," ++ param

     AtPCmd { enable, pitch, roll, gaz, yaw } ->
         let enableNum = if enable then "1" else "0"
             suffix = intercalate "," . map (show . floatToInt) $
                      [pitch, roll, gaz, yaw]
         in "AT*PCMD=" ++ show num ++ "," ++
            enableNum ++ "," ++ suffix

     AtFTrim -> "AT*FTRIM=" ++ show num
     AtCtrl a b -> "AT*CTRL=" ++ show num ++ ","++ show a ++","++show b
     AtConfig a b -> "AT*CONFIG="++ show num ++ ",\""++ a++ "\",\"" ++ b ++ "\""

-- | Helper function to convert a float
floatToInt :: Float -> Int32
floatToInt = fromIntegral . floatToWord
