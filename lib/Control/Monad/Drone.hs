module Control.Monad.Drone where

import Control.Monad.State
import Control.Monad.Trans
import Control.Concurrent
import Network.Socket
import Data.Binary.Get
import qualified Network.Socket.ByteString as NBS
import qualified Data.ByteString as BS
import Data.Word
import Control.Monad.Except

import Robotics.ArDrone.Control
import Robotics.ArDrone.NavDataParser

maxListenQueue = 1
droneIp="192.168.1.1"
navPort=5554
ctrlPort=5556
byte = BS.singleton ( 1 :: Word8)

data DroneExceptions = ParseError | ProtocolError deriving (Show)

data DroneState = DroneState { seqNr :: Integer
                             , lastCommand :: AtCommand
                             , ctrlSocket :: Socket
                             , navDataSocket :: Socket }


type Drone a = ExceptT DroneExceptions (StateT DroneState IO) a

takeOff :: Drone ()
takeOff = do
  cmd $ toAtCommand TakeOff
  wait 4

land :: Drone ()
land = do
  cmd $ toAtCommand Land

stop :: Drone ()
stop = do
  cmd $ toAtCommand Stop

disableEmergency :: Drone ()
disableEmergency = do
  cmd $ toAtCommand DisableEmergency

--calibrate :: Drone ()
--calibrate = do
--  cmd $ toAtCommand Calibrate

ftrim :: Drone ()
ftrim = do
  cmd $ toAtCommand FTrim

flyForward :: Float -> Drone ()
flyForward v = do
  cmd $ toAtCommand $ Front v

flyBackwards :: Float -> Drone ()
flyBackwards v = do
  cmd $ toAtCommand $ Back v

flyLeft :: Float -> Drone ()
flyLeft v = do
  cmd $ toAtCommand $ MoveLeft v

flyRight :: Float -> Drone ()
flyRight v = do
  cmd $ toAtCommand $ MoveRight v

flyUp :: Float -> Drone ()
flyUp v = do
  cmd $ toAtCommand $ Up v

flyDown :: Float -> Drone ()
flyDown v = do
  cmd $ toAtCommand $ Down v

rotateClockwise :: Float -> Drone ()
rotateClockwise v = do
  cmd $ toAtCommand $ Clockwise v

rotateCounterClockwise :: Float -> Drone ()
rotateCounterClockwise v = do
  cmd $ toAtCommand $ CounterClockwise v

initNavaData :: Drone ()
initNavaData = do
  cmd $ AtCtrl 5 0
  cmd $ AtRef "0"
  cmd $ AtPCmd False 0.0 0.0 0.0 0.0
  cmd $ AtConfig "general:navdata_demo" "FALSE"
  cmd $ AtCtrl 5 0
  cmd $ AtRef "0"
  cmd $ AtPCmd False 0.0 0.0 0.0 0.0
  cmd $ AtRef "0"
  cmd $ AtPCmd False 0.0 0.0 0.0 0.0
  cmd $ AtConfig "general:navdata_options" "8"
  cmd $ AtRef "0"
  cmd $ AtPCmd False 0.0 0.0 0.0 0.0
  cmd $ AtCtrl 5 0
  cmd $ AtRef "0"
  cmd $ AtPCmd False 0.0 0.0 0.0 0.0

inc :: Drone ()
inc = do
  state <- get
  put $ state { seqNr = seqNr state + 1}

cmd :: AtCommand -> Drone ()
cmd atcmd = do
  n <- gets seqNr
  ctrlS <- gets ctrlSocket
  state <- get
  liftIO $ send ctrlS $ fromAtCommand atcmd $ fromIntegral n
  put $ state { lastCommand = atcmd}
  inc

getNavData :: Drone NavData
getNavData = do
  navS <- gets navDataSocket
  msg <- liftIO $ NBS.recv navS 4096
  let navData = runGetOrFail parseNavData $ fromStrict msg
  case navData of
    Left _ -> throwError ParseError
    Right (_, _, n) -> return n

flushSocketBuffer :: Drone BS.ByteString
flushSocketBuffer = do
  navS <- gets navDataSocket
  msg <- liftIO $ NBS.recv navS 65536
  return msg

wait :: Double -> Drone ()
wait t
  | t >= 0.01 = do
    state <- get
    liftIO $ threadDelay 10000
    stayAlive
    inc
    wait ( t - 0.01 )
  | otherwise = return ()

stayAlive :: Drone ()
stayAlive = cmd =<< gets lastCommand

runDrone :: Drone a -> IO (Either DroneExceptions a)
runDrone d = do
  ctrlInfo <- getAddrInfo Nothing (Just droneIp) (Just $ show ctrlPort)
  let ctrlAddr = head ctrlInfo
  ctrlSocket <- socket (addrFamily ctrlAddr) Datagram defaultProtocol
  connect ctrlSocket (addrAddress ctrlAddr)

  navInfo <- getAddrInfo Nothing (Just droneIp) (Just $ show navPort)
  let navAddr = head navInfo
  navSocket <- socket (addrFamily navAddr) Datagram defaultProtocol
  connect navSocket (addrAddress navAddr)

  NBS.send navSocket byte

  evalStateT (runExceptT d) (DroneState 0 (AtCtrl 5 0) ctrlSocket navSocket)
