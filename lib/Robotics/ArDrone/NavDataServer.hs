module Robotics.ArDrone.NavDataServer where

import Network.Socket
import Control.Concurrent
import Data.IORef
import Data.Binary.Get
import qualified Network.Socket.ByteString as NBS
import qualified Data.ByteString as BS

import Robotics.ArDrone.NavDataParser

runServer :: Socket -> IORef (Maybe NavData) -> IO ()
runServer navSocket currentPacket = do
  msg <- NBS.recv navSocket 4096
  let navData = runGetOrFail parseNavData $ fromStrict msg
  case navData of
    Left (_, _, s) -> do putStrLn s
                         writeIORef currentPacket Nothing
    Right (_, _, n) -> writeIORef currentPacket $ Just n
  runServer navSocket currentPacket
