import Control.Monad.Drone
import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad.Trans

main :: IO ()
main = do
  runDrone $ do
    initNavaData
    mainLoop
  return ()

mainLoop :: Drone ()
mainLoop = do
  nd <- getNavData
  liftIO $ print nd
  wait 0.2
  mainLoop
