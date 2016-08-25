import Control.Monad.Drone
import Robotics.ArDrone.Control
import Control.Monad.Trans

main :: IO ()
main = runDrone $ do
  initNavaData
  mainLoop

mainLoop :: Drone ()
mainLoop = do
  nd <- getNavData
  lift $ print nd
  wait 0.2
  mainLoop
