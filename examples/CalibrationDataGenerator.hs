
import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad
import Control.Monad.Trans
import Control.Concurrent
import Data.MonadicStreamFunction
import Data.Maybe
import Data.Monoid
import Data.IORef
import Control.Monad.Except
import Data.Matrix

import Control.Monad.Drone
import Robotics.ArDrone.NavDataParser

data Matrix3x3 = Matrix3x3 { col1 :: Vector
                           , col2 :: Vector
                           , col3 :: Vector
                           } deriving (Show)


data DroneOrientation = BottomDown | CameraDown | RightSideDown deriving (Eq)

data ActiveThread = Main | Worker DroneOrientation


main :: IO ()
main = do
  ref <- newIORef Main
  forkIO $ droneCommunication ref
  putStrLn "Put the drone on a level surface and leave it there. Press enter key."
  _ <- getLine
  writeIORef ref $ Worker BottomDown
  waitForWorkerThread ref
  putStrLn "Now put the drone in a stable position with the camera facing downwoards. Press enter."
  _ <-  getLine
  writeIORef ref $ Worker CameraDown
  waitForWorkerThread ref
  putStrLn "Now put the drone in a stable position with right hand side of the drone [from the drones point pov] facing downwoards. Press enter."
  _ <- getLine
  writeIORef ref $ Worker RightSideDown
  putStrLn "Now wait while the calibration data is calculated."
  waitForWorkerThread ref
  putStrLn "Thanks for your patience."

droneCommunication :: IORef ActiveThread -> IO ()
droneCommunication session = do
  result <- runDrone $ do
    initNavaData
    orientation <- waitForMainThread session
    ensureOrientation orientation BottomDown
    flushSocketBuffer
    z <- calculateAvgVector
    liftIO $ writeIORef session Main
    orientation <- waitForMainThread session
    ensureOrientation orientation CameraDown
    flushSocketBuffer
    x <- calculateAvgVector
    liftIO $ writeIORef session Main
    orientation <- waitForMainThread session
    ensureOrientation orientation RightSideDown
    flushSocketBuffer
    y <- calculateAvgVector
    let vlist = [(vectorToList x), (vectorToList y), (vectorToList z)]
    let matrix = inverse $ transpose $ fromLists vlist
    return matrix
  case result of
    Left e -> liftIO $ putStrLn $ show e
    Right r -> case r of
      Left e2 -> putStrLn $ show e2
      Right m -> writeFile "calib_data.txt" $ show m
  writeIORef session Main

waitForMainThread :: IORef ActiveThread -> Drone DroneOrientation
waitForMainThread ref = do
  thread <- liftIO $ readIORef ref
  case thread of
    Main -> do wait 0.1
               waitForMainThread ref
    Worker o -> return o

waitForWorkerThread :: IORef ActiveThread -> IO ()
waitForWorkerThread ref = do
  threadDelay 500000
  thread <- liftIO $ readIORef ref
  case thread of
    Worker _ -> do waitForWorkerThread ref
    Main -> return ()

ensureOrientation :: DroneOrientation -> DroneOrientation -> Drone ()
ensureOrientation o1 o2 = if o1 == o2 then return () else throwError ProtocolError

calculateAvgVector :: Drone Vector
calculateAvgVector = do
  vs <- replicateM 1000 readAccVector
  let n = fromIntegral $ length vs
  let vecSum = foldr addVector (Vector 0 0 0) vs
  let vecResult = Vector ((x vecSum)/n) ((y vecSum)/n) ((z vecSum)/n)
  return vecResult

readAccVector :: Drone Vector
readAccVector = do
  navData <- getNavData
  stayAlive
  let accVector = fromMaybe (Vector 0 0 0) $ accelerometers <$> physMeasures navData
  return accVector

addVector :: Vector -> Vector -> Vector
addVector (Vector x1 y1 z1) (Vector x2 y2 z2) = Vector (x1 + x2) (y1 + y2) (z1 + z2)

vectorToList :: Vector -> [Float]
vectorToList (Vector x y z) = [x, y, z]
