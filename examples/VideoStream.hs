import Control.Monad.Drone

import Control.Concurrent
import Control.Monad.Trans

main :: IO ()
main = do
  runDrone WithoutVideo $ do
    initNavaData
    mainLoop
  return ()

mainLoop :: Drone ()
mainLoop = do
  stayAlive
  liftIO $ threadDelay 10000
  mainLoop
