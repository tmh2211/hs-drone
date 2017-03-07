import Control.Monad.Drone
import Robotics.ArDrone.Control  hiding (runDrone, main)
import Control.Monad.Trans

main :: IO ()
main = do
  runDrone WithoutVideo $ do
    takeOff
    wait 1
    flyForward 0.1
    wait 1
    flyBackwards 0.1
    wait 1
    land
  return ()
