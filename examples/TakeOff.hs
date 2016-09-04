import Control.Monad.Drone
import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad.Trans

main :: IO ()
main = do
  runDrone $ do
    takeOff
    wait 6
    land
  return ()
