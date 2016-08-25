import Control.Monad.Drone
import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad.Trans

main :: IO ()
main = runDrone $ do
  takeOff
  wait 6
  land
