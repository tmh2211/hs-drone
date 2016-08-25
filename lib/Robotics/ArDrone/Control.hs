{-# LANGUAGE NamedFieldPuns #-}

module Robotics.ArDrone.Control
where

import Network.Socket
import Control.Concurrent
import Control.Monad

import Data.List

import Data.Binary.IEEE754
import Data.Int

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

type SequenceNumber = Int

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

     MoveLeft speed -> AtPCmd True 0 speed 0 0
     MoveRight speed -> AtPCmd True 0 (-speed) 0 0

     Stop -> AtPCmd False 0 0 0 0

     FTrim -> AtFTrim
     DisableEmergency -> AtRef "290717952"

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


floatToInt :: Float -> Int32
floatToInt = fromIntegral . floatToWord
