
import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad
import Control.Monad.Trans
import Control.Concurrent
import Data.MonadicStreamFunction
import Data.Maybe
import Data.Monoid
import Data.IORef

import Control.Monad.Drone
import Robotics.ArDrone.NavDataParser

data Matrix3x3 = Matrix3x3 { col1 :: Vector
                           , col2 :: Vector
                           , col3 :: Vector
                           } deriving (Show)

data CalibState = Start  | BottomDown | CameraDown | SideDown | Calculating

main :: IO ()
main = do
  ref <- newIORef Start
  result <- newIORef $ Matrix3x3 (Vector 0 0 0) (Vector 0 0 0) (Vector 0 0 0)
  forkIO $ droneCommunication ref result
  putStrLn "Put the drone on a level surface and leave it there. Press enter key."
  _ <- getLine
  writeIORef ref BottomDown
  waitWhileCalculating ref
  putStrLn "Now put the drone in a stable position with the camera facing downwoards. Press enter."
  _ <-  getLine
  writeIORef ref CameraDown
  waitWhileCalculating ref
  putStrLn "Now put the drone in a stable position with right hand side of the drone [from the drones point pov] facing downwoards. Press enter."
  _ <- getLine
  writeIORef ref SideDown
  waitWhileCalculating ref
  m <- readIORef result
  putStrLn $ show m

waitWhileCalculating :: IORef CalibState -> IO ()
waitWhileCalculating ref = do
  threadDelay 500000
  v <- readIORef ref
  case v of
    Calculating -> waitWhileCalculating ref
    _ -> return ()

droneCommunication :: IORef (CalibState) -> IORef Matrix3x3 -> IO ()
droneCommunication ref result = do
  runDrone $ do
    initNavaData
    forever $ do
      value <- lift $ readIORef ref
      case value of
        Start -> do wait 0.2
        BottomDown -> do lift $ writeIORef ref Calculating
                         v <- calculateAvgVector
                         Matrix3x3 c1 c2 c3 <- lift $ readIORef result
                         lift $ writeIORef result $ Matrix3x3 c1 c2 v
                         lift $ writeIORef ref Start
        CameraDown -> do lift $ writeIORef ref Calculating
                         v <- calculateAvgVector
                         Matrix3x3 c1 c2 c3 <- lift $ readIORef result
                         lift $ writeIORef result $ Matrix3x3 v c2 c3
                         lift $ writeIORef ref Start
        SideDown -> do lift $ writeIORef ref Calculating
                       v <- calculateAvgVector
                       Matrix3x3 c1 c2 c3 <- lift $ readIORef result
                       lift $ writeIORef result $ Matrix3x3 c1 v c3
                       lift $ writeIORef ref Start

calculateAvgVector :: Drone Vector
calculateAvgVector = do
  let n = (150 :: Float)
  vs <- replicateM 150 readAccVector
  let vecSum = foldr addVector (Vector 0 0 0) vs
  let vecResult = Vector ((x vecSum)/n) ((y vecSum)/n) ((z vecSum)/n)
  return vecResult

readAccVector :: Drone Vector
readAccVector = do
  navData <- getNavData
  let accVector = fromMaybe (Vector 0 0 0) $ accelerometers <$> physMeasures navData
  return accVector

addVector :: Vector -> Vector -> Vector
addVector (Vector x1 y1 z1) (Vector x2 y2 z2) = Vector (x1 + x2) (y1 + y2) (z1 + z2)
