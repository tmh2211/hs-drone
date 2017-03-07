{-|
Module:         Robotics.ArDrone.NavDataServer
Description:    Server that receives Navdat from the drone and serves it to the application.
License:        GPS-2
Maintainer:     thomasmhergenroeder@gmail.com

This module starts a server that receives the navigational data that the drone
sends, parses it and serves it to the Haskell application using it.
-}
module Robotics.ArDrone.NavDataServer where

import Network.Socket
import Control.Concurrent
import Data.IORef
import Data.Binary.Get
import qualified Network.Socket.ByteString as NBS
import qualified Data.ByteString as BS

import Robotics.ArDrone.NavDataParser

-- | Starting the server.
runServer :: Socket -> IORef (Maybe NavData) -> IO ()
runServer navSocket currentPacket = do
  msg <- NBS.recv navSocket 4096
  let navData = runGetOrFail parseNavData $ fromStrict msg
  case navData of
    Left x -> do
      --putStrLn $ show x
      writeIORef currentPacket Nothing
    Right (_, _, n) -> writeIORef currentPacket $ Just n
  runServer navSocket currentPacket
