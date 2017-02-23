{-# LANGUAGE BangPatterns #-}
{-|
Module:         Control.Monad.Drone
Description:    High level communication with the drone
License:        GPL-2
Stability:      Tested on Windows 10

This moduleprovides the functionality to communicate with the drone and send
basic commands, like flying forward or receiving the navigational data. It also
provides the types needed and some helper functions to simplify the
implementation.
-}
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
import Data.Ratio
import Data.Time.Clock
import Data.Time.Clock.POSIX
import Control.Monad.Except

import Robotics.ArDrone.Control hiding (runDrone)
import Robotics.ArDrone.NavDataParser
import Robotics.ArDrone.NavDataConstants
import Robotics.ArDrone.NavDataServer
import Robotics.ArDrone.VideoStreamServer
import Util.MatrixParser

-- | The IP of the drone within the network
droneIp="192.168.1.1"
-- | Port for receiving navigational data. Uses UDP
navPort=5554
-- | Port where the video stream of the front camera goes out. Uses TCP
videoPort = 5555
-- | Port where the drone receives commands. Uses UDP
ctrlPort=5556
-- | Initialization byte for the navigational data port
byte = BS.singleton ( 1 :: Word8)
-- | Time before another command should be send to keep the connection alive in ms.
waitTime = 20

-- | Data flag that determines if the video stream server will run.
data DroneConfig = WithVideo | WithoutVideo deriving (Show)

-- | Exception that may occur when using the drone monad
data DroneExceptions
                  -- | This exception is thrown when the parser could not parse a packet of naviagtional data.
                  = ParseError String
                  -- | This error occurs when something with the communication is wrong.
                  | ProtocolError deriving (Show)

-- | This data type encapsulates the mutable drone state that is kept through the drone monad.
data DroneState = DroneState { seqNr :: Integer                         -- ^ Sequence number the next packet must have
                             , lastCommand :: AtCommand                 -- ^ The last command the user send
                             , lastTimeStamp :: Integer                 -- ^ The time stamp of the last command sent
                             , ctrlSocket :: Socket                     -- ^ UDP socket that connects to control port
                             , calibrationMatrix :: Matrix Float        -- ^ Calibration data for acceleration vector in form of a 3x3 matrix
                             , currentPacket :: IORef (Maybe NavData)   -- ^ This IORef holds the last packet of navigational data received
                             , imgBytes :: IORef (Maybe BS.ByteString)  -- ^ Holds the bytes of the last .png image received from the front camera
                             }

-- | The drone monad, the central data type of this library.
-- It is a moand transformer consisting of three moands.
-- At the top there is the ExceptT monad to handle errors.
-- At the second level is the StateT monad transformer that carries the mutable state of the drone around.
-- At the bottom is the IO monad to communicate with the system and over the network.
type Drone a = ExceptT DroneExceptions (StateT DroneState IO) a

-- | This function lets the drone take off. After the initial take off command it waits 4 seconds to make sure the drone started properly.
takeOff :: Drone ()
takeOff = do
  cmd $ toAtCommand TakeOff
  wait 4

-- | This function lands the drone right where it is.
land :: Drone ()
land = cmd $ toAtCommand Land

-- | This function resets the last command that was set and lets the drone hover where it is.
stop :: Drone ()
stop = cmd $ toAtCommand Stop

-- | After the drone was in emergency state, this function enables the drone again.
disableEmergency :: Drone ()
disableEmergency = cmd $ toAtCommand DisableEmergency

-- | This function sets the current plane as the reference for the drone.
ftrim :: Drone ()
ftrim = cmd $ toAtCommand FTrim

-- | This function lets the drone fly forward.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
flyForward :: Float -> Drone ()
flyForward v = cmd $ toAtCommand $ Front v

-- | This fucntion lets the drone fly backwards.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
flyBackwards :: Float -> Drone ()
flyBackwards v = cmd $ toAtCommand $ Back v

-- | This function lets the drone fly to the left.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
flyLeft :: Float -> Drone ()
flyLeft v = cmd $ toAtCommand $ MoveLeft v

-- | This function lets the drone fly to the right.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
flyRight :: Float -> Drone ()
flyRight v = cmd $ toAtCommand $ MoveRight v

-- | This function lets the drone fly up.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
flyUp :: Float -> Drone ()
flyUp v = cmd $ toAtCommand $ Up v

-- | This function lets drown sink towards the ground.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
flyDown :: Float -> Drone ()
flyDown v = cmd $ toAtCommand $ Down v

-- | This function lets the drone rotate clockwise.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
rotateClockwise :: Float -> Drone ()
rotateClockwise v = cmd $ toAtCommand $ Clockwise v

-- | This function lets the drone rotate counter clockwise.
-- It takes a value between 0 and 1 which determines how much of its power it will use.
rotateCounterClockwise :: Float -> Drone ()
rotateCounterClockwise v = cmd $ toAtCommand $ CounterClockwise v

-- | This function configures which navigational will be send from the drone.
-- It takes a list of NavDataOption and generates the bitfield that the drone will understand.
-- Demo data will always be send. Also the frequency is 200Hz.
configureNavDataOptions :: [NavDataOption] -> Drone ()
configureNavDataOptions [] = return ()
configureNavDataOptions xs = do
  let ints = map optionToInt xs
  let bitPos = map (shiftL (1 :: Int)) ints
  let mask = foldr (.|.) 1 bitPos
  cmd $ AtConfig "general:navdata_options" $ show mask
  cmd $ AtCtrl 5 0

-- | This function intializes the reception of the navigational data.
-- This musst be called once before you can parse the data and get it from the state.
initNavaData :: Drone ()
initNavaData = do
  cmd $ AtCtrl 5 0
  cmd $ AtConfig "general:navdata_demo" "FALSE"
  configureNavDataOptions [DEMO]
  cmd $ AtCtrl 5 0

-- | Helper function that increases the sequnce number for the next command.
inc :: Drone ()
inc = do
  state <- get
  put $ state { seqNr = seqNr state + 1}

-- | This function actually send the command to the drone and modifies
-- the drone state accordingly
cmd :: AtCommand -> Drone ()
cmd atcmd = do
  n <- gets seqNr
  ctrlS <- gets ctrlSocket
  state <- get
  liftIO $ send ctrlS $ fromAtCommand atcmd $ fromIntegral n
  time <- liftIO timeInMillis
  put $ state { lastTimeStamp = time}
  put $ state { lastCommand = atcmd}
  inc

-- | This function gets the last received and parsed navigational data packet
-- from the parser. It blocks until it there is something to get, so be sure to
-- call initNavData once before you call this function.
getNavData :: Drone NavData
getNavData = do
  ref <- gets currentPacket
  packet <- liftIO $ readIORef ref
  case packet of
    Nothing -> getNavData
    Just n -> do
      liftIO $ writeIORef ref Nothing
      return n

-- | This function waits the given time in seconds.
-- Furthermore it keeps the connection alive by resending the last command to
-- the dorne after a given time.
-- This function should also be called after you sent a moving command to the
-- drone, as the dorne only executes a command once.
wait :: Float -> Drone ()
wait t = do
  let ms = t * 1000
  currentT <- liftIO timeInMillis
  lastT <- gets lastTimeStamp
  let diffT = currentT - lastT
  when (diffT >= waitTime) stayAlive
  now <- liftIO timeInMillis
  let waitLeft = ms - realToFrac(now - currentT)
  when (waitLeft > 0) $
    wait $ waitLeft * 1000

-- | Helper function that resends the last command to keep the connection alive.
stayAlive :: Drone ()
stayAlive = cmd =<< gets lastCommand

timeInMicros :: IO Integer
timeInMicros = numerator . toRational . (* 1000000) <$> getPOSIXTime

timeInMillis :: IO Integer
timeInMillis = (`div` 1000) <$> timeInMicros

-- | Function that executes the monadic drone actions.
-- It takes the DroneConfig flag to decide if the videostream server is started.
runDrone :: DroneConfig -> Drone a -> IO (Either DroneExceptions a)
runDrone dc d = do
  -- Initialize control socket to be able to send commands.
  ctrlInfo <- getAddrInfo Nothing (Just droneIp) (Just $ show ctrlPort)
  let ctrlAddr = head ctrlInfo
  ctrlSocket <- socket (addrFamily ctrlAddr) Datagram defaultProtocol
  connect ctrlSocket (addrAddress ctrlAddr)

  -- Initialize navigational data socket to be able to receive sensor data.
  navInfo <- getAddrInfo Nothing (Just droneIp) (Just $ show navPort)
  let navAddr = head navInfo
  navSocket <- socket (addrFamily navAddr) Datagram defaultProtocol
  connect navSocket (addrAddress navAddr)

  -- Initialize navdata port with a byte with the value 1.
  NBS.send navSocket byte
  currentPacket <- newIORef $ Just emptyNavData
  -- STart server to receive navdata in another thread.
  forkIO $ runServer navSocket currentPacket

  -- Either read calibration matrix from file or initialize it with the identity
  -- matrix scaled by 1/-9.81 if no such file is present.
  !matrix <- readMatrixFromFile "calibData.txt"

  --Initialize video stream reception
  lastFrame <- newIORef Nothing
  case dc of
    WithVideo -> do
      -- If flag has Value WithVideo start the video server in another thread.
      forkIO $ runVideoServer lastFrame
      return ()
    WithoutVideo -> return ()

  -- Set now as the last time a command was send
  now <- timeInMillis

  -- Run the ExceptT monad to expose the underlying StateT transformer.
  -- Evaluate The StateT transformer after that and discard the final drone state.
  evalStateT (runExceptT d) (DroneState 0 (AtCtrl 5 0) now ctrlSocket matrix currentPacket lastFrame)
