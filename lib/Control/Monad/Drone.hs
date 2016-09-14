{-# LANGUAGE BangPatterns #-}
module Control.Monad.Drone where

import Control.Monad.State
import Control.Monad.Trans
import Control.Concurrent
import Network.Socket
import Data.Binary.Get
import qualified Network.Socket.ByteString as NBS
import qualified Data.ByteString as BS
import Data.Word
import Data.Bits
import Data.IORef
import Data.Matrix
import Control.Monad.Except

import Robotics.ArDrone.Control
import Robotics.ArDrone.NavDataParser
import Robotics.ArDrone.NavDataConstants
import Robotics.ArDrone.NavDataServer
import Util.MatrixParser

droneIp="192.168.1.1"
navPort=5554
ctrlPort=5556
byte = BS.singleton ( 1 :: Word8)

data DroneExceptions = ParseError String | ProtocolError deriving (Show)

data DroneState = DroneState { seqNr :: Integer
                             , lastCommand :: AtCommand
                             , ctrlSocket :: Socket
                             , calibrationMatrix :: Matrix Float
                             , currentPacket :: IORef (Maybe NavData)
                             }


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

configureNavDataOptions :: [NavDataOption] -> Drone ()
configureNavDataOptions [] = return ()
configureNavDataOptions xs = do
  let ints = map optionToInt xs
  let bitPos = map (shiftL (1 :: Int)) ints
  let mask = foldr (.|.) 1 bitPos
  cmd $ AtConfig "general:navdata_options" $ show mask
  cmd $ AtCtrl 5 0

initNavaData :: Drone ()
initNavaData = do
  cmd $ AtCtrl 5 0
  cmd $ AtConfig "general:navdata_demo" "FALSE"
  configureNavDataOptions [DEMO]
  cmd $ AtCtrl 5 0

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
  ref <- gets currentPacket
  packet <- liftIO $ readIORef ref
  case packet of
    Nothing -> getNavData
    Just n -> return n

wait :: Float -> Drone ()
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

  currentPacket <- newIORef $ Just emptyNavData

  forkIO $ runServer navSocket currentPacket

  !matrix <- readMatrixFromFile "calibData.txt"

  evalStateT (runExceptT d) (DroneState 0 (AtCtrl 5 0) ctrlSocket matrix currentPacket)
