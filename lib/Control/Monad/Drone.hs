module Control.Monad.Drone where

import Control.Monad.State
import Control.Monad.Trans
import Control.Concurrent
import Network.Socket
import Data.Binary.Get
import qualified Network.Socket.ByteString as NBS


import Robotics.ArDrone.Control
import Robotics.ArDrone.NavDataParser

droneIp="192.168.1.1"
navPort=5554
ctrlPort=5556

data DroneState = DroneState { seqNr :: Integer
                             , lastCommand :: AtCommand
                             , ctrlSocket :: Socket
                             , navDataSocket :: Socket }

type Drone a = StateT DroneState IO a

inc :: Drone ()
inc = do
  state <- get
  put $ state { seqNr = seqNr state + 1}

cmd :: AtCommand -> Drone ()
cmd atcmd = do
  n <- gets seqNr
  ctrlS <- gets ctrlSocket
  lift $ send ctrlS $ fromAtCommand atcmd $ fromIntegral n
  inc

getNavData :: Drone NavData
getNavData = do
  navS <- gets navDataSocket
  msg <- lift $ NBS.recv navS 4096
  let navData = runGet parseNavData $ fromStrict msg
  return navData

wait :: Double -> Drone ()
wait t
  | t > 0.1 = do
    lastCmd <- gets lastCommand
    state <- get
    lift $ threadDelay 100000
    cmd lastCmd
    put $ state { seqNr = seqNr state + 1 }
    wait ( t - 0.1 )
  | otherwise = return ()


runDrone :: Drone a -> IO a
runDrone d = do
  ctrlInfo <- getAddrInfo Nothing (Just droneIp) (Just $ show ctrlPort)
  let ctrlAddr = head ctrlInfo
  ctrlSocket <- socket (addrFamily ctrlAddr) Datagram defaultProtocol
  connect ctrlSocket (addrAddress ctrlAddr)

  navInfo <- getAddrInfo Nothing (Just droneIp) (Just $ show navPort)
  let navAddr = head navInfo
  navSocket <- socket (addrFamily navAddr) Datagram defaultProtocol
  connect navSocket (addrAddress navAddr)

  evalStateT d (DroneState 0 (AtCtrl 5 0) ctrlSocket navSocket)
