import Control.Monad.Drone
import Robotics.ArDrone.Control  hiding (runDrone, main)
import Control.Monad.Trans

main :: IO ()
main = do
  runDrone $ do
    takeOff
    wait 5
    flyForward 0.1
    wait 3
    flyBackwards 0.1
    wait 4
    land
  return ()
